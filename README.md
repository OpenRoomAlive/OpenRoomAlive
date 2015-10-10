DerpVision
===
FAQ
---
1. `arc install-certificate` failed to connect to server due to SSL error.

        Export self-signed SSL certificate from Chrome and save it as ./libphutil/resources/ssl/custom.pem

2. How do I setup `arc` on lab machines?

        git clone https://github.com/phacility/arcanist.git
        git clone https://github.com/phacility/libphutil.git
        echo "set path = ($path ~/work/arcanist/bin)" >> ~/.cshrc
        source ~/.cshrc
        cd ~/work/DerpVision
        arc which
