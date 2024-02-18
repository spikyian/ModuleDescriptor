
# Generators
This folder contains scripts for some modules that will generate a module descriptor file for the appropriate module.
A generator is useful where there is repeated information, e.g. the same content for channels 1 to 8, with tokens for simple substitution like variable number.
It is recommended the script is run with the output to the 'output' folder - as this is folder is ignored by source control
e.g. :

```./generate_CANMIO.sh > output/CANMIO-0D20-9z.json```

# Requirements
The generators are shell scripts. 
To run them you need ```bash``` which is provided with Linux systems.
For Windows system ```bash``` can be provided by [Cygwin](https://www.cygwin.com) and [Git Bash](https://git-scm.com/).