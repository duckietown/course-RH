



## Getting help


### `man` {#man}

This is an interface to the on-line reference manuals. Whenever you meet some unfamiliar commands, try use `man certain_command` before Googling. You will find it extremely clear, useful and self-contained.

## Looking around

### `cd` {#cd}

Go to the directory you want. If you just use:

    $ cd

then you will go to your home directory, i.e., /home/user/

To go up one directory use:

    $ cd ..
    
### `ls` {#ls}

List all the files and documents in the current directory. From `~/.bashrc`, we know some commonly used alias. See more by `man ls`.

-- `la` for `ls -a` which will list out all files and documents including the hidden ones(whose name starts with a dot).

-- `ll` for `ls -l` which will display Unix file types, permissions, number of hard links, owner, group, size, last-modified date and filename.

## Showing authority

### `sudo` {#sudo}

Whenever you want to modify system files, you will need `sudo`. Commonly touched system files including `/etc`, `/opt` and so on. Since most of you have followed the guideline to use passwordless sudo, I would recommend that make sure what you are doing with sudo before you execute the command, otherwise you may need to reinstall the system.


## `visudo` {#visudo status=draft}

TODO: to write

## Changing things around

### `cp` {#cp}

`cp fileA directoryB` will copy the file A to directory B. See more by executing `man cp`.

### `mv` {#mv}

`mv fileA directoryB` will move the file A to directory B. See more by executing `man mv`.
This command can be used to rename a file, to do so execute:
`mv fileA fileB`

### `mkdir` {#mkdir}

Make new directory. See more by `man mkdir`.

### `rm` {#rm}

Remove certain file. `rm -r` will also remove directories. More in `man rm`.

### `touch` {#touch}

Update the access and modification times of the input file to current time. See more by `man touch`.
It can be used to create empty documents, for example:
`touch duckie.txt` Will create an empty text file.

## Restarting

### `reboot` {#reboot}

This command must be executed as root. `sudo` required. This will reboot your laptop or Raspberry Pi. See more by `man reboot`.

### `shutdown` {#shutdown}

This command requires `sudo`. You can set a countdown to shutdown you machine. More by `man shutdown`.

 

## UNIX processes


### `cat` {#cat}

Cat some file will return you the content. More in `man cat`.


### `tee` {#tee}

Read from standard input and write to standard output and files. More on `man tee`.

## `truncate` {#truncate status=beta}

TODO: to write

## Linux disks and files {#linux-disk-and-files status=beta}

### `fdisk` {#fdisk}

TODO: to write


### `mount` {#mount}

TODO: to write

### `umount` {#umount}

TODO: to write


### `losetup` {#losetup}

TODO: to write


### `gparted` {#gparted}

TODO: to write


### `dd` {#dd}

TODO: to write


### `sync` {#sync}

TODO: to write


### `df` {#df}

TODO: to write
 
 



## Users and permissions {#users-and-permissions status=ready}

### `passwd` {#passwd}

Update password of the current user. Old password needed.

### `chmod` {#chmod}

`chmod` changes permission to a file or a directory. To make a file executable run:

    $ sudo chmod +x FILE

### `groups` {#groups status=beta}

TODO: to write

### `adduser` {#adduser status=beta}

TODO: to write

### `useradd` {#useradd status=beta}

TODO: to write


## Downloading {#download-utils status=beta}

## `curl` {#curl}

TODO: to write


## `wget` {#wget}

TODO: to write

 
