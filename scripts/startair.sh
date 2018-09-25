#/bin/bash

python loadconfig.py ../example/retr.json &
../build/cmavnode -H 8000

while true; do
	sleep 1
	python loadconfig.py ../example/retrsafe.json &
	../build/cmavnode -H 8000
done
