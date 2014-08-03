find . -iname *.cpp | xargs sed '/setSync/{
	N
	/setSynchronizedInputs(true);\n$/d 
}' -i | grep foo -B2 -C2
find . -iname *.cpp | xargs sed '/setSynchronizedInputs(true)/d' -i | grep foo -B2 -C2

