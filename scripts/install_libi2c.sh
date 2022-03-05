cd ~/code
git clone https://github.com/amaork/libi2c
cd libi2c
sudo make libi2c.a
sudo cp libi2c.a /usr/lib
cd include
sudo cp -r i2c /usr/include