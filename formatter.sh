# formatter.sh
find src/ examples/ -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c -o -iname *.ino | xargs clang-format -i
