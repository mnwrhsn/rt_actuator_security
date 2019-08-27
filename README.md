

### Implementation of I/O-Peripheral Security for RTS


##### Related publication: 

Monowar Hasan, Sibin Mohan. 2019. Protecting Actuators in Safety-Critical
IoT Systems from Control Spoofing Attacks. In 2nd Workshop on the Internet
of Things Security and Privacy (IoT S&P’19), November 15, 2019, London,
United Kingdom.

Link: https://arxiv.org/abs/1908.09444


#### Clone the repo (in the host machine):

* tested on Ubuntu 16.04 LTS

` git clone --recurse-submodules https://github.com/mnwrhsn/rt_sec_io_tz.git`

#### Build and Copy TrustZone codes to Raspberry Pi:

1. Download toolchains for build:

```
cd /build
make -j2 toolchains
```

2. Now Make:

```
make -j `nproc`
```

3. Make Client, examples, etc. (this is required since we use a different Makefile -- see `/build/Makefile`):

```
make optee-os optee-client optee-examples
```

See the details in OPTEE Guide: https://optee.readthedocs.io/building/gits/build.html#build

**NOTE:** _Step 2 and 3 can be simplified by running: `./do_build_for_sdcard.sh`_

4. Check `copy_to_sdcard.sh` for details of copying the compiled code to Raspberry Pi

5. Locally bulid TEE-Supplicant (due to glibc compatibility)

[this step needs to be performed in RPi3. Install necessary packages as required]

Download the optee-client:
`svn export https://github.com/mnwrhsn/rt_sec_io_tz/trunk/optee_client`

first `cd` to `optee_client`
and then:
```
make clean
cmake .
make
sudo make install
```

6. In the RPi3 -> load supplicant: `sudo tee-supplicant &`

**NOTE:** _Step 5 and 6 is optional if only running our kernel side TEE APIs_

**NOTE:** Any Linux kernel config can be updated using `/build/kconfigs/rpi3.conf`
However this will require to make a clean compilation (e.g. `make linux-clean` and then `make linux` from the `/build` directory)
