# ModuleDescriptor
VLCB/CBUS Module descriptor format &amp; examples

# documents
Please see the latest Module Descriptor File Format document for detailed infomation

# file naming
The naming of the module descriptors follows the following format

\<module name\>-\<module identity\>-\<version\>.json

Where the module identity is the combination of the manufacturer & module id in hex

And the version number is the major & minor number as reported by the module

an example for the CANMIO module is CANMIO-A520-3d.json

Note that for development work, a manufacturer code of 13 is recommended (0x0D), allowing any module id to be used for an individuals own use

# work_in_progress
This folder contains module descriptors that whilst are valid files, are likely to be incomplete, and not yet validated as complete

# generators
This folder contains scripts for some modules that will generate a module descriptor file for the appropriate module

A generator is useful where there is repeated information, e.g. the same content for channels 1 to 8, with tokens for simple substitution like variable number

Please see the readme in the generator folder for more information


