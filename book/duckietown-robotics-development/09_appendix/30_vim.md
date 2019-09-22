
# VIM {#vim status=ready}

To do quick changes to files, especially when logged remotely,
we suggest you use the VI editor, or more precisely, VIM ("VI iMproved").

## External documentation

See: [A VIM tutorial](http://www.openvim.com/).

## Installation

Install like this:

    $ sudo apt install vim


## `vi` {#vi status=beta}

TODO: to write

## Suggested configuration

Suggested `~/.vimrc`:

    syntax on
    set number
    filetype plugin indent on
    highlight Comment ctermfg=Gray
    autocmd FileType python set complete isk+=.,(


<!-- autocmd FileType python set complete+=k~/.vim/syntax/python.vim isk+=.,( -->
## Modes in VIM {#vim-modes status=ready}

There are 3 different modes is VIM. Command mode, insert mode and visual mode. Let us look at them one by one.

### Command mode

By default you are in the command mode when entering a file. Pressing <kbd>Esc</kbd> takes you back to the command mode when you are in insert mode or visual mode. When you are in this mode, you can give different kinds of commands. They have the commonality of starting with a double dot `:`.


Some helpful commands are:

* `:x`  saving and exiting the file

* `:q!` exit the file without saving

* `:q` exit

* `:help`  jump into the general help guide of VIM

* `:help keyword` look for help on a specific keyword

* `/keyword` find keyword in the document (note that there is no `:` used here)

### Insert mode

If you want to edit your file you can press <kbd>i</kbd>. You are now in insert mode. This can be verified by looking at the bottom left of your terminal. You should see `--INSERT--` written there. Now is the time to make all the changes you desire. When you are done, press <kbd>Esc</kbd> to leave the insert mode and enter the command mode again.

### Visual mode

Last but not least, there is the visual mode. In visual mode you can select text as you would typically do with your mouse but here you are using the keyboard. When you selected the text you can for example copy it by pressing <kbd>y</kbd> and paste it by pressing <kbd>p</kbd>.


## Indenting using VIM

Use the <kbd>&gt;</kbd> command to indent.

To indent 5 lines,  use
 <kbd>5</kbd>
 <kbd>&gt;</kbd>
 <kbd>&gt;</kbd>.

To mark a block of lines and indent it, use <kbd>V</kbd>.

For example, use <kbd>V</kbd><kbd>J</kbd><kbd>J</kbd><kbd>&gt;</kbd> to indent 3 lines.

Use <kbd>&lt;</kbd> to dedent.

## Tools

It can be hard to memorize all of the useful commands by heart. [Here](https://vim.rtorr.com/) is a cheat sheet that will help you look up things.
