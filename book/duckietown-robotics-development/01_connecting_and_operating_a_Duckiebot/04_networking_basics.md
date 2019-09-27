# Networking basics {#networking-basics status=ready}

Excerpt: Learn the basics of networking, to make sure you can connect to the Duckiebot.

Networking is extremely vital in Duckietown. And we don't mean the networking events where duckies socialize (these
 are pretty fun), but rather the computer networks between the bots, your computers and the rest of the Duckietown
  equipment. These networks allow us to do some pretty cool stuff, like controlling your Duckiebot from your laptop
   or creating a centralized observation center that combines the video streams of all watchtowers. Networking's usefulness is only comparable with its complexity. Indeed, this is often the source of most confusion and problems for Duckietown newbies. That is why we will try to clarify as many things as we can from the very beginning.

<div class='requirements' markdown='1'>

  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup).

  Requires: [Duckiebot initialization](+opmanual_duckiebot#setup-duckiebot).

  Results: Fundamental networking knowledge.

</div>

<minitoc/>


## Why do we care about networking in the first place? {status=ready}

Your Duckiebot, just like your computer or your phone, is a network device and you connect to it through the network. You probably want to control it without having to attach a screen, a keyboard and a mouse to it, that would defeat the whole "autonomy" goal. In more complex projects, one computer can also be used to control dozens of devices at a time. And in one of the most challenging undertakings that we have attempted so far, we connect 50+ watchtowers into a single mega-hive. All this is enabled by smartly configured computer networks!

## How do computer networks work? {status=ready}

A local network is setup with a _router_ at the center, that allows all devices that connect to it to find each other and communicate. The role of the router is to direct (route) packages from a sender to a receiver. In big networks you cannot physically connect all devices to a single router. In this case, you can use _switches_ to combine the network traffic from a number of devices onto a single connection to a router. The router must know which device is which and where to find it. To facilitate their communication, the router and the rest of the devices use _IP_ and _MAC_ addresses.

The [MAC address](https://en.wikipedia.org/wiki/MAC_address) is related to your hardware itself, to your computer (or more accurately, to the network interface). This means that it remains the same even if you move to the other end of the world and connect to a different network. If your computer supports both a WiFi and an Ethernet connection, then each one has a different MAC address. The MAC address is of the form: `0d:12:2c:a7:0d:27`, with each symbol being a hexadecimal (`0-9 + a-f`). More importantly, MAC addresses are unique: there is no other device in the world with the same MAC address as the WiFi adapter in your laptop. You can consider it as a citizen number: it is unique personal identifier. That makes MAC addresses extremely useful for routing messages reliably.

While MAC addresses have the benefit of stability, they are very clumsy to work with, imagine that every time you want to send a letter to your friend you need to write down their citizen number. And also imagine you are the mailman: it is very different to deliver mail if you don’t know where the person lives. Computers use IP addresses to handle these problems.

The [IP address](https://en.wikipedia.org/wiki/IP_address) of a device is relative to the network it lives in. It is
 a sequence of numbers that are uniquely mapped to devices inside the network. It is coded on 32 bits. Most home
  networks use the range of IP from `192.168.1.1` to `192.168.1.255`, so you may have seen the numbers before.  The
   structure of the IP address shows the hierarchical nature of the network architecture. This address will change as
    soon as you change network, and it is assigned by the network administrator. Typically this is handled by a [DHCP
    ](https://en.wikipedia.org/wiki/Dynamic_Host_Configuration_Protocol) server which, in most home networks is part
     of the router. In a local network, all addresses use the same subnetwork, which means that the first 24 bits of it are the same. If my IP is `192.168.1.23`, then my subnetwork is `192.168.1.![xyz]`. This makes it easy to determine which devices are on the same local network as me, as then the router can directly deliver my messages. If you are trying to connect to a device outside your local network (e.g., on the Internet), the router will need to find a way to deliver the message to it.

This concept is actually quite important. Your router will give you the address of any device on your _local_ network, such that you can connect to it, but does not work for resources on the Internet, for example, `docs.duckietown.org`. Therefore, instead, it acts as an intermediary between your device and the Internet. The technical term for that is _gateway_. The router will mask any request that comes from you as if it comes from the router itself, and once it gets a reply from the remote server, it forwards that back to your device.

Even though using IP addresses is very convenient for computers, humans do not handle them that well. They change from time to time and are hard to memorize. Instead, we prefer to name our devices with memorable names such as `quackabot` or `duckiecar`. These names are called _hostnames_ and you should have picked one for your Duckiebot when you initialized it. In Duckietown, we mostly use the hostnames for connecting to devices. However, the ability to find a device by hostname is non-standard and requires a protocol called _multicast DNS_ ([mDNS](https://en.wikipedia.org/wiki/Multicast_DNS)). 

Note: This mDNS protocol works by default on most home or office networks, but is blocked on large corporate networks like the ones of universities. If you have issues connecting to your Duckiebot thourgh the hostname, that is the most likely reason and you should first check with your network provider if mDNS is indeed blocked.

#### Network utilities {#exercise:ex-ifconfig}

Now we will discuss some useful tools that can help understand the network on which you are.

There is nothing simpler than finding your hostname: simply type `hostname` in a terminal. Now, make sure you are connected to a network first. 

We can use the `ifconfig` command to find some properties of this network. Open a terminal and type the command `ifconfig`. You might be missing the package that provides this command. If that is the case, install it and try again.

The `ifconfig` command outputs a few paragraphs, one for each network interface. You typically will find one called something like `wlan0` (your wireless interface) and another one called `eth0` (your Ethernet interface). Look at the one through which you are connected at the moment. After the keyword `inet` you should see your IP address and after the keyword `ether` or `HWadress` you should get the MAC address of this interface.

Can you determine what is your sub-network? How many devices can you put on this sub-network?

<end/>

Now that you know what your network is, it is time to explore the devices on it. There are many ways to do this. If you know about a device that should be connected, like your Duckiebot, then you can directly try to find it. To do so, you can try to ping it. This will just "poke" the device to see if it is on the network and it is responsive to the poking. You can ping by IP address and a hostname. Pinging by IP address always works if a device is connected to the network. Pinging by hostname requries that mDNS is enabled, therefore if that fails it could mean that either your device is not connected, or that the mDNS traffic is being blocked on your network.

#### Ping {#exercise:ex-ping}

Open a terminal. Run `ping ![hostname]`, where `hostname` is your Duckiebot’s hostname. Does it work? What is the output? Now try `ping ![hostname].local` instead. Does this work? For the router to find a device with its hostname, it needs to know that the hostname is in the local network, not somewhere else on internet. In contrast, try to ping a server outside of the local network: `ping google.com`. You can stop pinging the Duckiebot by pressing `CTRL-C`.

Now, when you pinged your Duckiebot, did you notice that there was an IP address in the output? Is it yours? No! It is the IP of the Duckiebot! You can now use this IP address and try pinging with it. Do you need to add the `.local` this time? Can you figure out why?


This part will be very important for a lot of the things you will do in Duckietown. When a command involving your Duckiebot doesn't work, the first thing to try is to ping it and make sure it is still accessible.

#### NMap {#exercise:ex-nmap}

We can now investigate what is on our network by using one of the many network mapping tools that exist out there. Keep in mind that depending on the network and the devices on it, you might not be able to see every device and every parameter.

Since you know your IP address, you also know your sub-network. Using the tool `nmap` , we are going to search the whole sub-network. Try to run `nmap -sP ![YOUR IP]/24` in a terminal. The `/24` part tells `nmap` to keep the 24 first bits the same in its search. If you don’t put it, then nmap will search the complete space of address (which are the monstrous 2^32 addresses).

The output should give you the list of all devices connected to your network, with their IP addresses and most of the time their hostnames. This way, you found your hostname and its IP, as well as other potentially present Duckiebots or computers.




## Connecting to your Duckiebot {status=ready}

Now that we know what our local network is and how it works, we can this information to gain access to Duckiebots. The industry standard way of connecting to remote devices is a protocol known as _SSH_ (Secure SHell). Then name describes it quite well: just in the same way that you can run shell commands on your computer in the terminal you can run shell commands, in a secure way, on a remote device. In this case, the remote device will be your Duckiebot.


#### SSH {#exercise:ex-ssh}

Let’s connect to our Duckiebot via SSH. Open a terminal and type `ssh ![username]@![hostname].local`. The username and hostname should be the ones you supplied when you flashed your card. If you didn’t set a username, then it should be the default value of `duckie`. If you are prompted to enter a password, again use the one you set when flashing, or if you didn’t use the default `quackquack` password.

Now your terminal is not in your computer anymore but on the Duckiebot. Did the text before the place where you can enter you command change? Why? What do these things there mean?

You should now be in a shell in the Duckiebot. Try to move around with terminal commands like `cd` and `ls`, as explained in the terminal basics. Verify that these are not the directories and files you find on your computer. They actually are the ones on your robot.

Repeating the steps from one of the previous exercises, find the MAC address of your Duckiebot.

Once you are ready, you can exit the session on the Duckiebot and return to your computer by simply typing `exit` or by pressing `CTRL+D`.

<end/>

You can connect to your bot without having to type a password (maybe that was already the case). This is done by using SSH keys (a _private_ and a _public_ one). You don't know this yet, but when you flashed the SD card on your computer, it added an SSH key to your computer and to the Duckiebot. With this, the Duckiebot recognizes your computer and won’t ask for a password. On your computer, the key is in `~/.ssh`, and it is called `DT18_key_00`. If you in fact try to `ssh` in a Duckiebot on the network that was not flashed on your computer, you will have to know the password.

####  SSH keys {#exercise:ex-ssh-keys}

Open a new terminal and navigate to `~/.ssh` and open the file named `config`. What is in there? It is a list of know agents mapped with the key to use. When you run `ssh ![hostname]` ssh will directly use the key and the provided Linux username (`duckie` by default).

<end/>
