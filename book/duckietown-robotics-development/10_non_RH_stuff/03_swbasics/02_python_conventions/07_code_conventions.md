# Basic Python code conventions  {#code-conventions-python status=ready}

Excerpt: How to write Python code that looks like Python code

## The Black Python formatter

You should use [`black`, the standard Python formatter,][black] which will make your code
look like anyone else's code.

[black]: https://github.com/psf/black

## Tabs, spaces, indentation {#no-tabs}

Indentation is 4 spaces.

Never use tabs in Python files. The tab characters are evil. 

If you find pre-existing code that uses tabs, be *very* careful in changing them.
Do *not* use a tool to do it (e.g. "Convert tabs to spaces"); it will get it wrong.



## Line lengths {#max-line-length}

Lines should be below 85 characters.


Long lines are a symptom of a bigger problem. The problem here is that you do not know how to program well,
therefore you create programs with longer lines.

Do not go and try to shorten the lines; the line length is just the symptom. Rather, ask somebody to take a look at the code and tell you how to make it better.


## The encoding line

All files must have an encoding declared, and this encoding must be `utf-8`:

```python
# -*- coding: utf-8 -*-
```

## Sha-bang lines

Executable files start with:

```python
#!/usr/bin/env python
```

## Comments

Comments refer to the next line.

Comments, bad:

```python
from std_msgs.msg import String # This is my long comment
```

Comments, better:

```python
# This is my long comment
from std_msgs.msg import String
```

## Imports

Do not do a star import, like the following:

```python
from rostest_example.Quacker import *
```

Rather, import each symbol by name:

```python
from rostest_example.Quacker import MySymbol, Another
```


## Logging  

Do not use `print` for logging.

Rather, use the `logging` library:

```python
# initialize the logger
import logging
logger = logging.getLogger('my program')
logger.setLevel(logging.DEBUG)

# then, later
logging.info("info message")
logging.error("error message")
logging.warning("warning message")
``` 

## Exceptions 

Exceptions are good. Throw them all the time! You will regret if you don't.

Get in the habit of catching and re-raising using the `from` keyword:

```python

try:
    some_function()
except BaseException as e:
    msg = 'Some function failed while I was doing this thing.'
    raise Exception(msg) from e
```

Python will show the full stack trace, telling you what exception caused what other.

## `sys.exit`

It is extremely rare that calling `sys.exit` is the right thing to do. You should use exceptions instead. 
