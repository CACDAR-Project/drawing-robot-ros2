#!/usr/bin/env sh
# Converts ROS2 log data timestamps from stdin to more readable format


while IFS= read -r string; do
    timestr=`echo $string | egrep -o "\[[0-9]+\.[0-9]+\]"`
    if [[ ! $timestr ]] then
        continue
    fi
    timestamp=${timestr#"["}
    timestamp=${timestamp%"]"}
    sec=`echo $timestamp | cut -d "." -f 1`
    nano=`echo $timestamp | cut -d "." -f 2`
    #nanos=$(( $nano /  1000000000 ))
    #sec=$(( $nanos + $sec ))
    timestamp=`date -d @"$sec" +"%Y-%m-%d-%H:%M:%S"`
    #printf 'Got sec:"%s"\n' "$sec"
    #printf 'Got nano:"%s"\n' "$nano"
    #printf 'Got timestamp:"%s"\n' "$timestamp"
    #head=`echo $string | awk -F'$timestr' '{print $1}'`
    #tail=`echo $string | awk -F'$timestr' '{print $2}'`
    #printf '%s[%s]%s\n' "$head" "$timestamp" "$tail"
    echo "${string/"$timestr"/"[${timestamp}.${nano}]"}"
done
