JSON=$*
if [ $JSON == '' ]; then
   JSON=*.json
fi

for filename in $JSON; do
    [ -e "$filename" ] || continue
    # echo $filename
    python -m json.tool $filename > beautiful_$filename
done
