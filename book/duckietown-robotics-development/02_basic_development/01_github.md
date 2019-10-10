# Git and GitHub {#git-github status=ready}

Excerpt: Learn how to use Git and Github.

Working on software in a group is great for development, but it automatically brings many pitfalls and issues.
How to handle code that has been modified at the same time by two members of the group? How to keep an eye on what other members write in the code? How to keep enough history of the code to be able to go back to a stable version when something bad was added? How to do that when a few hundred people work on the same code and not go crazy. The answer is simple: **code versioning tools**. These tools allow communities to swiftly handle these issues. The most used one, and the one we will use, is **git**.


<div class='requirements' markdown='1'>

  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)
  
  Results: Know how to extensively use a code versioning tools, git
  
</div>


<minitoc/>

## Learning git {status=ready}

Git is a great tool, that is **mandatory** to anyone doing any sort of code. Learning how to use it is essential.

#### Git tutorial {#exercise:git}

To learn how to use all of git's functionalities, complete [this tutorial](https://learngitbranching.js.org/). 

## What is github {status:ready}

*"GitHub is a code hosting platform for version control and collaboration. It lets you and others work together on projects from anywhere."* (source : github.com)

Github is where all the code is stored. It provides tools to handle pull requests, issues, and much more. The [duckietown organization github page](https://github.com/duckietown) hosts all relevant code. It is comprised of many different repositories.

## Being a good git citizen {status:ready}

Knowing how to use git is the first step. The second step, which is of the same importance, is knowing how to use it well.

### Commits

- Commits need to be **granular**: One commit contains on fix, or one function. It cannot have two new functions, and three bug fix. This means that it is better to do too many commits that not enough. This is helpful when doing cherry picks, or when checking out a previous version of the code.
- Commits need to have **meaningful messages**: The message of the commit should describe its content.

### Branches, forks, pull request and peer review

If you are going to work on a new function, but are not sure yet how it is going to go, then you cannot work on the master branch. This master branch needs to only receive code that has been tested, reviewed and approved by the team.

You then have **two solutions**:

- **Branching** On the main remote, you can branch out of the master branch, as explained in the above tutorial. Please give a relevant name to the branch (example : *"devel-new-flying-function"*). On repositories that you and a small team use a lot, this is the best option.
- **Forking** You can fork the main repo into you own workspace, and work from here. On repositories that are used by a lot of people, or that you very rarely will modify, this is the best option.

No matter the chosen solution, you then do your work, commit it, and then push it to github. On github, your branches will appear in your repository. When you feel like it is ready to be integrated to the master branch, you can open a [pull request](https://help.github.com/en/articles/about-pull-requests). This will allow your co workers to see the modifications you made.

**What you need to do:**

- Check that you are not committing wrong things by error.
- Provide a clear description of your work
- explain why it is relevant
- test it before opening the pull request, and explain that the test worked
- assign relevant co-workers to review the code

**What the reviewers need to do (all in the github interface):**

- Go through the modified code
- Comment directly on lines that raise questions and doubts
- Propose modifications
- And then, when all conversation are resolved, approve and merge the pull request

A pull request must never be approved and merged by the person who submitted it. Peer review is one of the most important part of software development. Not only it does allow for error proofing, but it also allows for someone to make a code suggestion alone. This way the code can be easily discussed and improved, even when it was functional to start with.
