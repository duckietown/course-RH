
## Git advanced tips

### Delete branches


Delete local:

    $ git branch -d ![branch-name]

Delete remote:

    $ git push origin --delete ![branch-name]


Propagate on other machines by doing:

    $ git fetch --all --prune


### Shallow clone

You can clone without history with the command:

    $ git clone --depth 1 ![repository URL]
