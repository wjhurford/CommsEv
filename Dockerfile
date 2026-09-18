# CommsEv in a browser.
#
#   docker build -t commsev .
#   docker run --rm -p 5800:5800 -v "$PWD/runs:/app/runs" commsev
#   -> open http://localhost:5800
#
# The base image runs an X server and a web (noVNC-style) viewer, and starts
# whatever /startapp.sh launches inside it. The Console is unchanged: this
# is the same PySide6 application, drawn in a browser tab instead of a window.
# Ubuntu 22.04 is deliberate - it is ROS 2 Humble's platform, so the ROS
# bridge can join this image later without changing the base.
FROM jlesage/baseimage-gui:ubuntu-22.04-v4

ENV APP_NAME="CommsEv Console" \
    KEEP_APP_RUNNING=1 \
    DISPLAY_WIDTH=1920 \
    DISPLAY_HEIGHT=1080 \
    DARK_MODE=1 \
    PYTHONUNBUFFERED=1 \
    QT_QPA_PLATFORM=xcb \
    PIP_ROOT_USER_ACTION=ignore \
    LANG=C.UTF-8 LC_ALL=C.UTF-8 DEBIAN_FRONTEND=noninteractive

# Python plus the shared libraries Qt needs on a bare X server. Nothing else.
#
# The base image ships a stripped-down /etc/passwd and /etc/group (no `staff`
# group, and name lookup can fail for `root` itself), and fontconfig's
# installer runs `chown root:staff`. So the two entries are guaranteed first,
# name lookup is pinned to those files, and the result is printed so a failure
# here is readable in the build log.
RUN grep -q '^root:'  /etc/passwd || echo 'root:x:0:0:root:/root:/bin/sh' >> /etc/passwd; \
    grep -q '^root:'  /etc/group  || echo 'root:x:0:'  >> /etc/group; \
    grep -q '^staff:' /etc/group  || echo 'staff:x:50:' >> /etc/group; \
    grep -q '^passwd:' /etc/nsswitch.conf 2>/dev/null \
      || printf 'passwd: files\ngroup: files\nshadow: files\n' >> /etc/nsswitch.conf; \
    echo "lookup check: $(getent passwd root) / $(getent group staff)" \
    && apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip git \
        libgl1 libegl1 libglib2.0-0 libfontconfig1 libdbus-1-3 \
        libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
        libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 \
        libxcb-xfixes0 libxcb-cursor0 libxrender1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt
COPY . .

# The test suite is the acceptance test for the image: an image that cannot
# pass it is not shipped. Runs headless.
RUN QT_QPA_PLATFORM=offscreen python3 tests/test_all.py > /dev/null

COPY docker/startapp.sh /startapp.sh
RUN chmod +x /startapp.sh && mkdir -p /app/runs && chmod -R a+rwX /app

VOLUME ["/app/runs"]
EXPOSE 5800 5900
