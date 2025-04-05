#!/bin/bash
find . -maxdepth 1 -regex '.*\.\(cpp\|hpp\|c\|h\)' > files.txt
while IFS= read -r file; do
  echo "$file"
  clang-format -i "$file"
done < files.txt
rm files.txt