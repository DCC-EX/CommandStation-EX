# Contributing

Thanks for considering contributing to our project. Here is a guide for how to get started and and a list of our conventions. We will also walk you through the Github command line and Desktop commands necessary to download the code, make changes, and get it included in a next version of the software.

Before contributing to this repository, please first discuss the change you wish to make via issue, or any other method with the owners of this repository before making a change. 

Find us on our website at https://dcc-ex.com, on our Discord https://discord.gg/y2sB4Fp or on Trainboard: https://www.trainboard.com/highball/index.php?threads/dcc-update-project-2020.130071/

# Git client

Firstly, in order to work with our code, you will require access to a Git client.

If you have a Linux based system (including macOS), you more than likely have Git installed already, so probably don’t need to install it.

However, for Windows, you will need to install Git, so follow the instructions in the [Pro Git book (https://git-scm.com/book/en/v2/Getting-Started-Installing-Git "Pro Git book").

Once you’ve installed Git, you will need to set some variables to tell Git who you are when committing code to a repository.

To do this, open a command prompt and run the following two commands, where user is your GitHub username, and email is the email address associated with your GitHub account (the same commands apply across all operating systems):

`git config --global user.name <user>`
`git config --global user.email <email>`

# Development Environment

We recommend using PlatformIO IDE for VSCode. If you haven't yet used it, it is an easy to learn and easy to use IDE that really shines for embedded development and the Arduino based hardware we use. For more information go to https://platformio.org/

* Download and install the latest version of the Arduino IDE
* Download and install the latest version of Visual Studio Code from Microsoft
* Run VSCode and click on the "extensions" icon on the left (or press `<Ctrl> + <Shift> + <x>`). Install "PlatformIO IDE for VSCode" and the "Arduino Framework" support

If you don't see C/C++ by Microsoft installed in the list, install that too. We also recommend installing the Gitlens extension to make working with Git more productive.

If you wish to use VSCode as your default editor for Git, open a command prompt and run the following command:

git config --global core.editor "code --wait"

You may ask if you can use the Arduino IDE, Visual Studio, or even a text editor and the answer is "of course" if you know what you are doing. Since you are just changing text files, you can use whatever you like as long as your commits and pull requests can be merged in GitHub. However, it will be much easier to follow our coding standards if you have an IDE that can automatically format things for you.

# Arduino IDE

If you’re already using DCC-EX, then it’s likely you’ve already installed the Arduino IDE.

However, if you haven’t, it’s recommended to have this installed, even if using our recommended editor (VSCode). It’s handy to see the same interface that most of our users can see, as they will almost certainly be using the Arduino IDE, and providing support is so much easier when you can use the same tools as the users.

To get up and running with the Arduino IDE, follow the Getting started with Arduino products page.

# Coding Style Guidelines

We have adopted the Google style guidelines. In particular please make sure to adhere to these standards:

1. All header files should have #define guards to prevent multiple inclusion.
2. Use Unix style line endings
3. We indent using two spaces (soft tabs)
4. Use Braces as shown in the code

To set your soft tabs:

1. Click the Settings cog icon in the bottom left corner and select “Settings” (alternatively, press <Ctrl> + “,”).
2. Locate “Editor: Tab size” (should be about the fourth item down).
3. Set this to “2”, and close the “Settings” tab which will auto-save the change.

For more information just check our code or read https://google.github.io/styleguide/cppguide.html#C++_Version

## Using the Repository

1. Clone the repository on your local machine
2. Create a working branch using the format "username-featurename" ex: "git branch -b frightrisk-turnouts"
3. Commit often from the "source control" icon (shft-cntrl-G). Add a description of the change.
4. Push your changes to our repository.
5. When you are ready, issue a pull request for your changes to be merged into the main branch

Git Bash users will be familiar with typing "git add ." and then "git commit -m "description of your changes" and lastly "git push". VSC handles all that for you with menus or mouse clicks.

## Pull Request Process

1. Ensure any install or build dependencies are removed before the end of the layer when doing a build.

## Code of Conduct

Be Nice

### Enforcement

Contributors who do not follow the be nice rule in good faith may face temporary or permanent banishment from the kingdom as  determined by other members of the project's leadership as the traditional tar and feathering was deemed "inappropriate".

## How Can I Contribute?

The DCC-EX Team has several projects and sub teams where you can help donate your expertise. See the sections below for the project or projects you are interested in.

### Documentation
### EX-CommandStation
### EX-WebThrottle
### Web Support
### Organization/Coordination

Be sure to see any updates or more details posted to our web page here: https://dcc-ex.com/about/contributing/software.html
