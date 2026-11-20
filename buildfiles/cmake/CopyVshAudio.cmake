# Copies the optional vsh_audio folder (it holds sys_audio.sprx, the audio server module that firmware without its own copy needs)
# next to the emulator. The folder is not part of the repository, it is filled once by hand in <repo>/bin/vsh_audio/.
# Runs at build time: cmake -DSRC=<source folder> -DDST=<destination folder> -P CopyVshAudio.cmake
# Does nothing when the source folder does not exist.
if(EXISTS "${SRC}")
    file(COPY "${SRC}/" DESTINATION "${DST}")
endif()
