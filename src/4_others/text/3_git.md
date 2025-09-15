# Git - Version Control System

Git is a distributed system for versioning and managing backups of source code. However, Git also works well for versioning any kind of text. The primary motivation for teaching Git in this course is the fact that Git is the most widely used version control system in the commercial sphere today, and there is a vast array of Git-based online version control services available on the web.

---

## Basic Terminology

Let’s define some basic terms to ensure we’re on the same page.

### Repository (repo)

A set of versioned files and records of their history. If the repository is stored on our computer, it is called a local repository (local repo). If it is stored on another machine, it is referred to as a remote repository (remote repo).

### Cloning

Downloading a repository from a remote repo. Cloning occurs when the repository does not yet exist on the local machine.

### Snapshot

The state of the repository at a specific point in its history.

### Diff

The difference between two snapshots, i.e., the changes in the state of versioned files.

### Commit

A record that contains a reference to the previous and next snapshot, as well as the diff between them. Each commit has a unique 20-byte hash that identifies it within the repository.

### Push

Uploading new commits to the remote repository.

### Fetch

Downloading commits from a remote repo to the local machine. Fetching is done when the local repository is already cloned but does not have the latest commits downloaded.

### Branch

A sequence of interconnected commits. By default, every repository has one branch (typically named "master" or "main"). If multiple features are being developed simultaneously, these developments can be divided into separate branches and merged back into the main branch once the feature is complete.


## How Git Works

The primary function of Git is versioning text files. It is important to note that Git is NOT suitable for versioning binary files. When developing a program and using Git for version control, you should always version source code only, never compiled executable files (binaries).

Git also enables highly efficient collaboration among multiple people working on the same project (repository). Developers can work together or individually on separate branches. However, a key rule is that two people must not overwrite the same line of code in two different commits, as this will cause a conflict. A general recommendation is that two people should avoid modifying the same file.

Unlike SVN, Git is a decentralized system. This means there is no superior, central repository or server. All repositories have the same functionality, can maintain the full history of the project, and can seamlessly communicate with all other clones. In practice, however, there is often a repository that acts as a central point for sharing commits between developers, commonly referred to as "origin".

It is important to note that any repository can download the complete history from the origin. In the event of an origin failure, no data is lost, as each developer has a complete copy of the repository on their computer.

---

### Typical Workflow with Git:

1. A repository is created on the server for the project.
2. Developers clone the repository to their local machines. From their perspective, the server is referred to as "origin".
3. Developers work on their local machines, creating code and committing changes.
4. At the end of the day, each developer pushes their daily commits to the origin.
5. The next morning, each developer fetches the commits from their colleagues made the previous day.

---

## Installing Git on Linux

If you are using a Debian-based distribution, Git can be installed using the following commands:

```bash
sudo apt install git
```

or

```bash
sudo snap install git
```


## Command Overview

### git init

Initializes a repository, turning a regular folder in the file system into a repository. A repository differs from a regular folder because it contains a hidden `.git` folder that stores the repository's history.

```bash
git init     # Initializes a repository
```

---

### git add

Adds changes made since the last commit to the index. The index is a staging area where changes are prepared for the next commit. This allows selective inclusion of changes in a commit.

```bash
git add myfile.txt     # Adds changes made to 'myfile.txt' to the index
git add .              # Adds all current changes to the index
```

---

### git commit

Creates a new commit derived from the last commit in the current branch. Includes changes (diffs) staged in the index.

```bash
git commit -m "Commit message"     # Creates a new commit in the current branch
```

---

### git checkout

Switches between snapshots.

```bash
git checkout .          # Reverts the branch to the last commit, discarding all changes
git checkout abcdef     # Switches to the state after commit 'abcdef'
git checkout master     # Switches to the last available commit in the 'master' branch
```

---

### git clone

Creates a local clone of a remote repository. No need to initialize with `git init`, as repository metadata is automatically downloaded along with the content.

```bash
git clone https://remote_repo_address.git     # Clones the repository to the local machine
```

---

### git remote

Manages connections to remote repositories.

```bash
git remote -v                                            # Lists the configuration of remote repositories
git remote add origin https://remote_repo_address.git    # Adds a remote alias named 'origin'
git remote remove origin                                 # Removes the 'origin' alias
```

---

### git push

Uploads new commits from the local repository to the remote repository.

```bash
git push origin master     # Pushes new commits from the 'master' branch to the remote repository
```

---

### git fetch

Downloads commits from the remote repository to the local repository. These commits are not automatically merged into the current branch.

```bash
git fetch origin           # Fetches all new commits from all branches of the 'origin'
git fetch origin master    # Fetches new commits for the 'master' branch from the 'origin'
```

---

### git merge

Creates a new commit in the current branch by merging changes from another branch, combining all their changes.

```bash
git merge cool_branch        # Merges the changes from 'cool_branch' into the current branch
```

---

### git pull

Combines `git fetch` and `git merge`. Commonly used to pull changes from a remote repository. It fetches commits from the remote repository and then merges them into the current branch.

```bash
git pull origin master        # Fetches and merges commits from 'master' branch of 'origin'
```

---

### git diff

Displays the difference between two snapshots (commits).

```bash
git diff abcdef 012345        # Shows the difference between commits 'abcdef' and '012345'
```

---

### git status

Shows the current state of changes since the last commit, including changes already staged in the index.

```bash
git status        # Displays the current state of changes
```

---

### git log

Displays a chronological history of commits along with their metadata (timestamp, commit message, hash, etc.).

```bash
git log        # Displays the history of the current branch
```

---

### git stash

Saves and retrieves changes to/from a stack. Useful when you realize you are working on the wrong branch. Changes can be stashed, allowing you to switch branches and reapply the changes later.

```bash
git stash        # Saves changes to the stack and reverts the branch to its state after the last commit
git stash pop    # Retrieves changes from the stack and applies them to the current state
```

## Exercise

### Basic Operations

1. Create a repository.
2. Create two text files in the repository and write a few lines in each.
3. Add the changes to the index and then commit them.
4. Edit one of the files and commit the changes.
5. Edit the second file and commit the changes.
6. Create an account on [GitHub](https://github.com) and create a new repository there.
7. Add the remote repository as "origin" to your local repository and push the changes to the origin.
8. Verify the repository's contents in the GitHub web interface.
9. On another location on your computer, or on a different computer, clone the repository you just pushed.
10. In the new clone, make a change, commit it, and push it to the origin.
11. In the original folder, pull the new commits from the origin.
12. Use the `git log` command to view the commit history.

---

### Conflict

An example of what happens when two developers change the same code.

1. Following the steps from the previous exercise, create two copies of the repository on the same computer or two different computers, both with the same origin on GitHub.
2. In the first clone, modify a specific line in a file, commit the change, and push it to the origin.
3. In the second clone, modify the same line, commit the change, and try to push (this will result in an error).
4. A conflict has been created. Two conflicting changes occurred at the same point in the repository's branch history.
5. Resolve the conflict by pulling from the origin in the second clone where the push failed.
6. Open the file containing the conflict. The conflict will be marked with special syntax:
   ```
   <<<<<<< local_change
   =======
   change_from_origin
   >>>>>>>
   ```
   Choose the desired version, remove the conflict markers, and save the file. The conflict is now resolved.
7. Run the `git commit` command without additional parameters to commit the resolved conflict. An automatic commit message will indicate that this is a conflict resolution.
8. Push the new commit to the origin, then pull it into the original repository.
9. Use the `git log` command to view the commit history.


## Other Resources

 - Git Cheat Sheet: https://education.github.com/git-cheat-sheet-education.pdf
 - Atlassian Git Tutorials: https://www.atlassian.com/git/tutorials
 - Official Git Documentation: https://git-scm.com/doc
 - Oh Shit, Git!? A helpful guide: https://ohshitgit.com/

