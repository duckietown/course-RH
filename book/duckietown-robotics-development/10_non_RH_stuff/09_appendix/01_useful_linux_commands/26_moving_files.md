## Moving files between computers  {#moving-files}

### `scp` {#scp status=beta}

The Secure Copy (scp) copies files from different hosts using ssh for data transfer.


### Download a file with SCP {#howto-download-file-with-scp}

To download a file named `file.txt` which is on `hostname` use this command:

    laptop $ scp ![hostname]:/path/to/file.txt /path/where/you/whish/.

to download `file.txt` to `/path/where/you/whish/`. If you want to download in the current directory simply use:

    laptop $ scp ![hostname]:/path/to/file.txt .

### `rsync` {#rsync status=beta}

TODO: to write
