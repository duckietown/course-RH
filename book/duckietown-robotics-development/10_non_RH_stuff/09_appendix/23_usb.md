# Mounting USB drives {#mounting-usb status=ready}

First **plug in the USB drive** nothing will work if you don't do that first. Now ssh into your robot. On the command line type:

    duckiebot $ lsusb

you should see your Sandisk USB drive as an entry. Congrats, you correctly plugged it in

    duckiebot $ lsblk

Under name you should see `sda1`, with size about 28.7GB and nothing under the `MOUNTPOINT` column (if you see something under `MOUNTPOINT` congrats you are done.

If it is not there already, make the directory to mount to:

    duckiebot $ sudo mkdir /data/logs

Next mount the drive

    duckiebot $ sudo mount -t vfat /dev/sda1 /data/logs -o umask=000

Test by running `lsblk` again and you should now see `/data/logs` under `MOUNTPOINT`

## Unmounting a USB drive {#unmount-usb}

    duckiebot $ sudo umount /data/logs
