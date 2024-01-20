sed -i -e 's/<p><\/p>/<script src="\/networktables\/networktables.js"><\/script><p><\/p>/g' dist/index.html
sed -i -e 's/<p><\/p>/<script src="\/networktables\/utils.js"><\/script><p><\/p>/g' dist/index.html
sed -i -e 's/<p><\/p>/<script src="\/networktables\/camera.js"><\/script>/g' dist/index.html
sed -i -e "/<\!--FL-->/rsrc/asset/wheel.svg" dist/index.html
sed -i -e "/<\!--FR-->/rsrc/asset/wheel.svg" dist/index.html
sed -i -e "/<\!--RL-->/rsrc/asset/wheel.svg" dist/index.html
sed -i -e "/<\!--RR-->/rsrc/asset/wheel.svg" dist/index.html