# Become a Docker Power-User {#docker-poweruser status=ready}

Excerpt: Learn how to make the most of Docker containers.

We already introduced in [](#docker-basics) what Docker containers are and how you can start them and do basic operations. Recall that a Docker container is a closed environment and any change you do there cannot affect your host system or other containers. This can be great if you want to protect your laptop from possible mischief coming from inside a container, but at the same time limits what you can do with it. Thankfully, Docker has some very powerful ways to interact with your system and the outside world.

<div class='requirements' markdown='1'>
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)

  Requires: [Docker basics](#docker-basics)

  Results: Advanced knowledge of using Docker images and containers
</div>

<minitoc/>


## Getting data in and out of your container {status=ready}

Docker provides a few ways to extract and import files from and to a container. We will look only at [volume mounting](https://docs.docker.com/storage/volumes/) as it is the most used and versatile way. In the simplest terms, mounting a volume to a container essentially means that you make a directory on your host machine available in the container. Then, you can think of these two directories as perfect copies of each-other: if you change something in one of them, it will be changed in the other as well. Therefore, if your container needs some data or configuration files to operate properly, or if you need to export your results out of it, volume mounting is the way to go. So, how does it work?

You can use `docker run` with the `-v host_dir:container_dir` option. Here `-v` is a shortcut for `--volume`. This specifies that `container_dir` in the container will be replaced with `host_dir` from your computer. Give it a try:

#### Docker volume mounting {#exercise:ex-docker-volumemounting}

Run a new Ubuntu container where you mount your home directory in the container’s home directory:

    laptop $ docker run -it -v ~:/home ubuntu

In bash `~` is a shortcut for your home directory (`/home/your_username`). Now if you check which files are in the container’s home directory by running `ls /home` you’d see the files you have on your host machine. Try to change one of them (hopefully one not that important file) or to create a new one with, for example, `touch test.txt`. The easiest way to modify a simple file is to append a string to its content with something like `echo "hello" >> test.txt`. Check in your host home folder if the changes appear there as well. Now do the opposite: make a change in your host and observe if there’s a corresponding change in the container. To visualize the content of a simple file you can use the command `cat test.txt`.

<end/>


## Docker and networking {status=ready}
The default [network environment](https://docs.docker.com/network/) of a Docker container (a bridge network driver) gives your container access to the Internet but not much more. If you run, for example, a web server in the container, you wouldn't be able to access it from your host. This is not ideal for us as most of the Duckietown code-base actually uses similar technologies to connect the various parts of the code.

However, by adding `--network host` to the `docker run` command, we can remove the network isolation between the container and the Docker host and therefore, you can use the full range of networking capabilities that your host has within the convenient environment in the container.


## Handling devices {status=ready}
The Docker containers do not have access to the devices on your computer by default. Yup, if you put your code in a container it cannot use the camera, wheels and LEDs of your Duckiebot. No fun, right? Thankfully, just like with the network, Docker has a solution for that! You can manually allow each device to be available to your container or you can allow all of them by simply passing the `--privileged` option to `docker run`. You will see that option being often used in Duckietown.


## Other fancy option {status=ready}

Docker provides many more options for configuring your containers. Here’s a list of the most common ones:

<div figure-id="tab:docker-run-tab" markdown="1">
  <style>
    td:nth-child(2) {
    white-space: pre;
   }
  </style>
  <col3 class="labels-row1" >
    <span>Short command </span>
    <span>Full command</span>
    <span>Explanation</span>
    <span>`-i`</span>
    <span>`--interactive`</span>
    <span>Keep STDIN open even if not attached, typically used together with `-t`.</span>
    <span>`-t`</span>
    <span>`--tty`</span>
    <span>Allocate a pseudo-TTY, gives you terminal access to the container, typically used together with `-i`.</span>
    <span>`-d`</span>
    <span>`--detach`</span>
    <span>Run container in background and print container ID.</span>
    <span></span>
    <span>`--name`</span>
    <span>Sets a name for the container. If you don't specify one, a random name will be generated.</span>
    <span>`-v`</span>
    <span>`--volume`</span>
    <span>Bind mount a volume, exposes a folder on your host as a folder in your container. Be very careful when using this.</span>
    <span>`-p`</span>
    <span>`--publish`</span>
    <span>Publish a container's port(s) to the host, necessary when you need a port to communicate with a program in your container.</span>
    <span>`-d`</span>
    <span>`--device`</span>
    <span>Similar to `-v` but for devices. This grants the container access to a device you specify. Be very careful when using this.</span>
    <span></span>
    <span>`--privileged`</span>
    <span>Give extended privileges to this container. That includes access to **all** devices. Be **extremely** careful when using this.</span>
    <span></span>
    <span>`--rm`</span>
    <span>Automatically remove the container when it exits.</span>
    <span>`-H`</span>
    <span>`--hostname`</span>
    <span>Specifies remote host name, for example when you want to execute the command on your Duckiebot, not on your computer.</span>
    <span></span>
    <span>`--help`</span>
    <span>Prints information about these and other options.</span>
  </col3>
  <figcaption><code>docker run</code> options</figcaption>
</div>

### Examples {nonumber}

Set the container name to `joystick`:

    --name joystick

Mount the host's path `/home/myuser/data` to `/data` inside the container:

    -v /home/myuser/data:/data

Publish port 8080 in the container as 8082 on the host:

    -p 8082:8080

Allow the container to use the device `/dev/mmcblk0`:

    -d /dev/mmcblk0

Run a container on the Duckiebot:

    -H duckiebot.local
