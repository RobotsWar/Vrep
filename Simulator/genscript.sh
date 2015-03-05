#!/bin/bash

echo "#!/bin/bash" > $2
echo "" >> $2
echo "cd $1/Vrep/; ./vrep.sh ../Scenes/Soccer_Field.ttt" >> $2
chmod +x $2
