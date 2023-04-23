#!/bin/bash
cd $1
baglist=baglist.txt
>$baglist
files=$(ls *.bag)
for bagname in $files
do
 echo $bagname >> $baglist
done
exit 0
