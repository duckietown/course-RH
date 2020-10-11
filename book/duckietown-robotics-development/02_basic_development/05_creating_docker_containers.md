# Creating Docker containers {#creating-docker-containers status=ready}

Excerpt: Learn how to create your first Docker container.

We spent a lot of time looking at how to use Docker containers and the image that they start from. But that still leaves a very important question open: how can you make your own image? Now you will have the opportunity to make your first image that will do some basic computer vision processing on your Duckiebot!

<div class='requirements' markdown='1'>
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)

  Requires: [Duckiebot initialization](+opmanual_duckiebot#setup-duckiebot)

  Requires: [Docker basics](#docker-basics)

  Requires: [Docker poweruser skills](#docker-poweruser)

  Results: Advanced knowledge of using Docker images and containers.
</div>

<minitoc/>


## Where do Docker containers come from? {status=ready}

So far we saw that you can get a Docker image from the [DockerHub](https://hub.docker.com/) by knowing its name. How do these images get on DockerHub? Well, the simple answer is that you [register](https://hub.docker.com/signup) an account and then similarly to git, you can push one of your images to DockerHub. And how do you create an image in the first place?

A simple, though rarely practiced way is to convert a container in which you have made some changes into a new image. This can be done through the docker commit command. However, as this is not the preferred mode of operation we won’t discuss it further. But you can find more details in the [official documentation](https://docs.docker.com/engine/reference/commandline/commit/).

The more popular and accepted way is to build an image from a "recipe", called a Dockerfile. A Dockerfile is a text file that specifies the commands required to create a Docker image, typically by modifying an existing container image using a scripting interface. They also have special keywords (which are always CAPITALIZED), like `FROM`, `RUN`, `ENTRYPOINT`, and so on. For example, create a file called Dockerfile with the following content:

```Dockerfile
FROM ubuntu
RUN touch new_file1
CMD ls -l
```

The first line above defines the base image on top of which we will build our container. The second line simply executes the Linux command `touch new_file1` which creates a new file with this name. And the last line is the default command that will be run when the container is started (unless the user provides a different command).

Now, to build the image we can simply run:

    laptop $ docker build -t my_first_container:v1 .

The last part of this command denotes the directory (called _context_) which contains your Dockerfile. The `.` shorthand refers to the current directory.

You should see something like:

```
Sending build context to Docker daemon  2.048kB
Step 1/3 : FROM ubuntu
 --- ea2f90g8de9e
Step 2/3 : RUN touch new_file1
 --- e3b75gt9zyc4
Step 3/3 : CMD ls -l
 --- Running in 14f834yud59
Removing intermediate container 14f834yud59
 --- 05a3bd381fc2
Successfully built 05a3bd381fc2
Successfully tagged my_first_container:v1
```

Now run the command `docker images` in your terminal, and you should see an image called `my_first_container` with tag `v1`:

```
$ docker images
REPOSITORY          TAG        IMAGE ID         CREATED          SIZE
my_first_container  v1         05a3bd381fc2     2 seconds ago    88.9MB
```

An interesting observation is that the container size is `88.9MB`. Now, instead of needing to carry around a `88.9MB` file, we can just store the `4KB` text file and rest assured that all our important setup commands are contained within. In a sense, a whole OS, with our custom file inside is compressed to 3 lines of code.

Now, similarly to before, we can simply run:

```
$ docker run -it my_first_container:v1
total 64
drwxr-xr-x   2 root root 4096 Mar  7  2019 bin
drwxr-xr-x   2 root root 4096 Apr 24  2018 boot
drwxr-xr-x   5 root root  360 Sep 21 18:45 dev
drwxr-xr-x   1 root root 4096 Sep 21 18:45 etc
drwxr-xr-x   2 root root 4096 Apr 24  2018 home
drwxr-xr-x   8 root root 4096 May 23  2017 lib
drwxr-xr-x   2 root root 4096 Mar  7  2019 lib64
drwxr-xr-x   2 root root 4096 Mar  7  2019 media
drwxr-xr-x   2 root root 4096 Mar  7  2019 mnt
-rw-r--r--   1 root root    0 Sep 21 18:41 new_file1
drwxr-xr-x   2 root root 4096 Mar  7  2019 opt
dr-xr-xr-x 328 root root    0 Sep 21 18:45 proc
drwx------   2 root root 4096 Mar  7  2019 root
drwxr-xr-x   1 root root 4096 Mar 12  2019 run
drwxr-xr-x   1 root root 4096 Mar 12  2019 sbin
drwxr-xr-x   2 root root 4096 Mar  7  2019 srv
dr-xr-xr-x  13 root root    0 Sep 21 18:45 sys
drwxrwxrwt   2 root root 4096 Mar  7  2019 tmp
drwxr-xr-x   1 root root 4096 Mar  7  2019 usr
drwxr-xr-x   1 root root 4096 Mar  7  2019 var
```

Notice that as soon as we run the container Docker will execute the `ls -l` command as specified by the Dockerfile, revealing that `new_file1` was indeed stored in the image. However, we can still override `ls -l` by passing a command line argument: `docker run -it your/duck:v3 [custom_command]`.


## Environment variables and Docker containers {status=ready}

Environment variables are often used to control the behavior of one or more programs. As the name hints, these variables are associated with a particular (terminal) environment and are shared among processes. In fact, all processes started from an environment inherit its set of environment variables. If you are curious, you can check out the [Wikipedia](https://en.wikipedia.org/wiki/Environment_variable) article about them.

In bash you can set an environment variable with `export VAR_NAME=var_value`, and to check a variable’s current value use <code>echo \$VAR_NAME</code>. Python allows you to easily get the environment variable of the environment where the program was started in through the `os` module and its dictionary `os.environ['VAR_NAME']`.


#### Environment variables in Docker {#exercise:ex-docker-envvar}

Open a terminal and set a new environment variable `MY_VAR` with any value you like. Then start an interactive Python session in the same terminal and check the value of `MY_VAR` using the above function.

<end/>

In the Docker universe environment variables are particularly useful to configure a container when you run it. Imagine that your code can be run with different configuration variables (e.g. gain for the motors, exposure mode of the camera, etc.). Then you can set the value of this variable when you run the container, e.g.

    laptop $ docker run -e CAMERA_EXPOSURE='sport' my_fancy_camera:alpha

Then the Python code in the container can obtain the value you passed via the `os.environ` dictionary. In this way you make a single Docker image that can initialize containers with all sorts of configurations. Quite powerful, right?


## Guide to the Dockerfile keywords {status=ready}

Here are some of the most commonly used Dockerfile keywords. You will see them in many of the Duckietown Dockerfiles and you will often make use of them. You can find much more information and details on how to use them on [Docker’s official documentation](https://docs.docker.com/engine/reference/builder/#usage).

<div figure-id="tab:dockerfile-keywords" markdown="1">
  <style>
    td:nth-child(1) {
    white-space: pre;
   }
  </style>
  <col2 class="labels-row1" >
    <span>Keyword</span>
    <span>Usage</span>
    <span>FROM</span>
    <span>Designates the base image on top of which this container image will be built (every Dockerfile should have one, and only one FROM command)</span>
    <span>RUN</span>
    <span>Executes any shell command at build time </span>
    <span>CMD</span>
    <span>Executes any shell command at run time, unless the user specifies another command. This is the default command the container will execute when you call docker run. A Dockerfile should have at most one of these.</span>
    <span>ENV</span>
    <span>Sets an environment variable to a particular value. Can be overwritten with the `-e` option at runtime.</span>
    <span>COPY</span>
    <span>Copies file from the context (e.g. the folder where your Dockerfile is) to the container</span>
    <span>WORKDIR</span>
    <span>Changes the working directory for the commands that come after it</span>
  </col2>
  <figcaption>Dockerfile keywords</figcaption>
</div>


## Creating your first functional Docker image {status=ready}

Now that you know your way around Dockerfiles, it is time to finally build something meaningful that works on your Duckiebot! We are going to build a very basic vision system: we will try to measure how much of the image stream the camera sees is covered with pixels of a particular color.


#### Creating a color detector in Docker {#exercise:ex-docker-colordetector}

Note: The following exercise will use the camera on your robot, bear in mind that only one process can   access the camera at a time. Therefore, if there is another process on your bot that is already using the camera, your code will likely fail. Make sure that the `dt-duckiebot-interface` and any other container that can use the camera are stopped. You can use [Portainer](#exercise:portainer) to do that.

We will divide the image that the camera acquires into `N_SPLITS` equal horizontal sectors. `N_SPLITS` will be an environment variable we pass to the container. Think of it as a configuration parameter. The container should find which color is most present in each sector. Or alternatively you can look at the color distribution for each split. It should print the result in a nicely formatted way with a frequency of about 1Hz.

You can start your Dockerfile from `duckietown/dt-duckiebot-interface:daffy-arm32v7`. Most of the stuff you need should already be in there. Make a `requirements.txt` file where you list all your pip dependencies. We would expect that you would need at least `picamera` and `numpy`. Using a `requirements.txt` file is a good practice, especially when you work with big projects. The Dockerfile then copies this file and passes it to pip which installs all the packages you specify there. Finally copy your code in the container and specify it should be the starting command. Here’s an example Dockerfile: 

```Dockerfile
FROM duckietown/dt-duckiebot-interface:daffy-arm32v7

WORKDIR /color_detector_dir

COPY requirements.txt ./

RUN pip install -r requirements.txt

COPY color_detector.py .

CMD python3 ./color_detector.py
```

Make sure you understand what each single line is doing. Keep in mind that you might need to modify it in order to work for you.

Note: If you are using a DB-Beta, so a NVIDIA Jetson Nano powered Duckiebot, you will need to change the "dt-duckiebot-interface:daffy-arm32v7" tag to "dt-duckiebot-interface:jetson".

Working with the camera can sometimes be tricky so you can use this template for `color_detector.py` to get started:

```python
#!/usr/bin/env python3
import cv2
import numpy as np
from time import sleep

cap = cv2.VideoCapture(2)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    #Put here your code!
    # You can now treat output as a normal numpy array
    # Do your magic here

    sleep(1)
```


Once you have your `color_detector.py` file ready to be tested, you can build it directly on your bot by running:


    $ docker -H ![DUCKIEBOT_NAME].local build -t colordetector .

Do you remember what `-H` does? It takes the context (the folder in which you are) and ships it to the device specified by `-H` and build the container there. Once the container is built (typically it takes more time the first time), you can test it with:

    $ docker -H ![DUCKIEBOT_NAME].local run -it --privileged colordetector

Again there is the `-H` option (why?) and we also have the `--privileged` option. Do you remember what it does? Try to remove it and see what happens.

We omitted to mention what to do about a lot of implementation details which can significantly affect the performance of your color detector. For example, what should the value of `N_SPLITS` be? Should we consider the whole width of the image or just a central part? How many colors should we detect, which ones and what is the best way to do it? Should you use RGB or HSV color space? All this is left for you to decide. This is typically the case in robotics: you know what the final result should be, but there are multiple ways to get there and it is up to you to decide which is the best solution for the particular case. Experiment and find what makes your color detector really good. We recommend investing some time in this, as this color detector will be a building block in the next module.

<end/>


## Pushing to DockerHub {status=ready}

Say that you want to share your awesome color detector with your friend. How can you do that? You can of course repeat the same procedure as above, just replacing your Duckiebot’s name with theirs. But that is cumbersome and requires them to have the code. DockerHub makes all this much easier. It allows you to push your image to their repository and then anyone can directly use it. That is where all the base images you saw so far come from.

To do this, first make sure you have a DockerHub account. Let’s say your account name is `duckquackermann`. Then sharing your container with the world is as easy as building it under your account name:

    laptop $ docker -H ![DUCKIEBOT_NAME].local build -t duckquackermann/colordetector .

Then push it to DockerHub:

    laptop $ docker -H ![DUCKIEBOT_NAME].local push duckquackermann/colordetector

Note: You will probably have to first connect your Duckiebot's Docker client with your DockerHub account. So first open an [SSH connection](#exercise:ex-ssh) to the robot and then run `docker login` in it. You will be prompted to provide your DockerHub username and password. If you want to be able to push images directly from your laptop, you should do the same there.

After you've pushed your image to DockerHub your code can be executed on any single Duckiebot around the world with a single command:

    $ docker -H ![DUCKIEBOT_NAME].local run -it --privileged duckquackermann/colordetector
