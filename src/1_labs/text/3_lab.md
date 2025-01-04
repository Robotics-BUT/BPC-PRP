# Lab 3 - Git & C++ Project Template

## Git (1h 30min)

First, read the [Git Tutorial](../../3_others/text/3_git.md) chapter to get familiar with the workflow and commands

### Exercise

### Sign On 

Select any of the following free Git Services and register for it
 - [GitHub](https://github.com/)
 - [GitLab](https://about.gitlab.com/)
 - [Bitbucket](https://bitbucket.org/product/)

This server will serve as your **"origin"** (remote repository) for the rest of the **BPC-PRP** course.

The instructors will have access to all your repositories, including their history, and can monitor your progress, including who, when, and how frequently commits were made.

Create a repository on the server to maintain your code throughout the course.

---

### Team Exercise

As a team, complete the following steps:

1. One team member creates a repository on the server.
2. All team members clone the repository to their local machines.
3. One team member creates a "Hello, World!" program locally, commits it, and pushes it to the origin.
4. The rest of the team pulls the changes to their local repositories.
5. Two team members intentionally create a conflict by modifying the same line of code simultaneously and attempting to push their changes to the server. The second member to push will receive an error from Git indicating a conflict.
6. The team member who encounters the conflict resolves it and pushes the corrected version to the origin.
7. All team members pull the updated version of the repository. Each member then creates their own `.h` file containing a function that prints their name. Everyone pushes their changes to the server.
8. One team member pulls the newly created `.h` files and modifies the "Hello, World!" program to use all the newly created code. The changes are then pushed to the origin.
9. All team members pull the latest state of the repository.

## C++ Project Template (30 min)

Now it is time to create your main project for this course.

1. Create a project on the web page of your Git service.
2. Clone project to the local
3. Create the following project structure

```/bpc-prp-project-team-x
 |--docs
 | \--placeholder
 |--README.md
 |--CMakeLists.txt
 |--.gitignore
 |--include
 | \--<project_name>
 |   \--lib.hpp
 \--src
   |--lib.cpp
   \--main.cpp
```

4. Fill all required files
 - README.md is a brief description and how-to-use your project.
 - folder `docs` will be used later. Now just create file `placeholder`. 
 - Write some basic code into the `cpp` adn `hpp` files.
 - Fill the `.gitignore` file. It is used to inform `git` to ignore some files, folders or extensions, not to commit it into the repository.

```.gitignore
# Ignore build directories
/build/
/cmake-build-debug/
/cmake-build-release/

# Ignore CMake-generated files
CMakeFiles/
CMakeCache.txt
cmake_install.cmake
Makefile

# Ignore IDE-specific files (CLion and JetBrains)
.idea/
*.iml
```
5. 
6. Commit and push your project to the server and share it with other members of the team.
