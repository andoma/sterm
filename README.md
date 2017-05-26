# sterm

I got fed up with using screen for connecting to my serial devices so I quickly made sterm

Just go ahead and
```
make
sudo make install
```
... will install in /usr/local/bin by default.

Then just run

```
sterm
```

will default open /dev/ttyUSB0 with baudrate 115200

Device and baudrate can be passed as command line arguments:

```
sterm -d device -b baudrate
```

GLHF
