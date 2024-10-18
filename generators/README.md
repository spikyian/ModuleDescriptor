
# Generators
This folder contains scripts for some modules that will generate a module descriptor file for the appropriate module.
A generator is useful where there is repeated information, e.g. the same content for channels 1 to 8, with tokens for simple substitution like variable number.

# Requirements
The generators are shell scripts.
To run them you need ```bash``` which is provided with Linux systems.
For Windows system ```bash``` can be provided by [Cygwin](https://www.cygwin.com) and [Git Bash](https://git-scm.com/).

# Generating Module Descriptor Files
It is recommended the script is run with the output to the 'output' folder - as this is folder is ignored by source control
e.g. :

```sh generators/generate_CANPAN.sh > output/CANPAN-0D20-9z.json```

# Writing Generator Scripts

See existing generator scripts for ideas and solutions to tricky needs.

## Coding style
We try to write generator scripts that generate JSON code that is as human readable as
possible.
I.e. using indentation and reasonable spacing.

We also try to write the generators with a similar structure to the generated
JSON file. 
This makes the generator look like a template where variables are used as placeholders
for items to be filled in with variable content.
This style makes it easy to find the correct place in the generator script to fix 
any problems detected in the generated JSON file.

## Structure of the generator script
Most of the generators are built around "Here documents" which allow expansion of
variables within a text block. 
A "Here document" is an embedded piece of text that is fed as standard input to
a process.
Here we use a ```cat``` command to simply pass that text to the standard output.
An example:
```
cat << EOF
Some text.
EOF
```
Some blocks of text is repeated, for example for each input channel.
For these cases we use a loop that then processes one "Here document" for each input channel.
Example:
```
for channel in 1 2 3 4 5 6 7 8
do
  cat << EOF
Stuff for channel ${channel}.
EOF
done
```

## Comma separated lists
JSON is strict on commas between items in a list and there may not be any trailing comma.
There are many ways to solve this. 
The method used here is a trick with variable expansion which requires little code
in the "Here document".

An example with four channels may have elements generated this way:
```
ending[0]=','
ending[1]=''
cat <<EOF
"elements" : [
EOF
for channel in 1 2 3 4
do 
  cat << EOF
   { 
     ...
   }${ending[$(($channel == 4))]}
EOF
done
cat <<EOF
]
EOF
```
```ending``` is an array of two elements. The first is a comma and the second is empty.
The element for the channel is followed by selecting one of these elements. 
If ```channel``` is 4, i.e. the last channel, the expression is true which translates
to the number 1. 
This 1 is used as the index to the ```ending``` array which refers to the empty element. 
If ```channel``` is something else, then the expression is false and the index is zero
and the first element of ```ending``` is used, i.e. the element with a comma.

Note that for this construct we need to know the value of the last channel and put that
into the expression used as index into ```ending```.