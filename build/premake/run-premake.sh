#
# remove any generated build files
#
rm -rf ..\gmake
rm -rf ..\vs2019
rm -rf ..\vs2022
rm -rf ..\obj
#
# remove any compiled libraries
#
rm -rf ..\..\lib\*.lib
#
# create new build files files
#
./premake5-beta1 --os=linux --file=premake5.lua gmake

