for file in `ls *.dot`; do name=$(echo $file| cut -f1 -d'.'); dot -Tpng $file > $name.png; done
