git clone https://github.com/arunoda/meteor-streams drawerapp/packages/streams
cp aux/package.json drawerapp/packages/ros/.npm/package
cd drawerapp/packages/ros/.npm/package/
npm install
rm package.json
