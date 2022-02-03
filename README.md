
This is readme.md file


WHEN UNZIPPING FIRMWARE PACKAGE YOU: 
- must have gcc-arm-none-eabi installed
- have to install libnewlib-arm-none-eabi because of #include_next error (fatal error: stdint no such file or directory)
- had to unncoment -nosys-spec.nospec 

OTHER ISSUES: 
-specs=nosys.specs ... If you remove this you get error undefined reference to _exit
