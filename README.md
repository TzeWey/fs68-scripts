# fs68-scripts

Scripts to control the LEDs, Fans and Buzzer on the Asustor FS68 series NAS.

Verified on the FS6812X.

# Disclaimer

The `fs68-fand` values are tuned to run at the lowest fan RPM where there are no
extended periods of high CPU loads. **Adjust thresholds accordingly**

# Usage

Clone this repository to a folder in TrueNAS.

In this example, we will clone the repository to a dataset called `pool-fast/INTERNAL/fs68`:

```
cd /mnt/pool-fast/INTERNAL/fs68
https://github.com/TzeWey/fs68-scripts.git
```

## Update "Init/Shutdown Scripts"

Navigate to `System -> Advanced Settings -> Init/Shutdown Scripts`

Add the following entries:

| Type    | Description    | When      | Command/Script                                              | Enabled |
| ------- | -------------- | --------- | ----------------------------------------------------------- | ------- |
| Command | FS68 Pre-Init  | Pre Init  | /mnt/pool-fast/INTERNAL/fs68/fs68-scripts/fs68-pre-init.py  | Yes     |
| Command | FS68 Post-Init | Post Init | /mnt/pool-fast/INTERNAL/fs68/fs68-scripts/fs68-post-init.py | Yes     |
| Command | FS68 Shutdown  | Shutdown  | /mnt/pool-fast/INTERNAL/fs68/fs68-scripts/fs68-shutdown.py  | Yes     |

## Run `fs68-fand` in a privileged Docker container

```
services:
  fs68-fand:
    image: python:alpine3.21
    privileged: true
    restart: unless-stopped
    volumes:
      - /mnt/pool-fast/INTERNAL/fs68:/app
    entrypoint: ["python3", "/app/fs68-scripts/fs68-fand.py"]
```
