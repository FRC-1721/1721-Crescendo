sed -i -e 's/<p><\/p>/<script src="\/networktables\/networktables.js"><\/script><p><\/p>/g' dist/index.html
sed -i -e 's/<p><\/p>/<script src="\/networktables\/utils.js"><\/script><p><\/p>/g' dist/index.html
sed -i -e 's/<p><\/p>/<script src="\/networktables\/camera.js"><\/script>/g' dist/index.html

sed -i -e "/<\!--FL-->/rsrc/asset/wheel/FL.svg" dist/index.html
sed -i -e "/<\!--FR-->/rsrc/asset/wheel/FR.svg" dist/index.html
sed -i -e "/<\!--RL-->/rsrc/asset/wheel/RL.svg" dist/index.html
sed -i -e "/<\!--RR-->/rsrc/asset/wheel/RR.svg" dist/index.html
sed -i -e "/<\!--FLd-->/rsrc/asset/wheel/FLd.svg" dist/index.html
sed -i -e "/<\!--FRd-->/rsrc/asset/wheel/FRd.svg" dist/index.html
sed -i -e "/<\!--RLd-->/rsrc/asset/wheel/RLd.svg" dist/index.html
sed -i -e "/<\!--RRd-->/rsrc/asset/wheel/RRd.svg" dist/index.html