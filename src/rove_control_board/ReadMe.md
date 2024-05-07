# rove_control_board

## Setup

### Conditions
This was tested with a [UCAN](https://wiki.fysetc.com/UCAN/) board.
It was flashed with the candellight (a8a0757) firmware using [this web app](https://canable.io/updater/canable1.html).
Currently, as a temporary test rig, an Arduino Due was used as the microcontroller (repo will be added here shortly).

### Using serial [UNTESTED]
In order to access the serial port to communicate through can, you need to make give access to your local user to dialout group

If you run `ls -l /dev` while the usb to can converter is connected, you should get something like this:
```sh
crw-rw----   1 root  dialout 166,     0 Apr 30 11:07 ttyACM0
```
We can see that the port is part of the dialout group.
All you need to do is to edit `/etc/group`and add your userid to the dialout group. It should look like this:
```sh
...
dialout:x:20:<your id>
...
```
Where `<your id>` is your local user id.
Then restart your device. Logging out is not sufficient.

### Using socketcan [TESTED]
After plugging in the UCAN board, you should be able to see the can interface by running `ip a`. Note will not be shown in `/dev/`.

Then you can setup the interface by running this:
```sh
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

You can then ensure the interface is up by running `ip a`.