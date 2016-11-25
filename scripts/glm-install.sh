#!/bin/bash

# Travis CI has a really old version of GLM that is missing
# tvec2 among other things that Frontier.Engine needs

git clone https://github.com/g-truc/glm.git /tmp/glm
cd /tmp/glm
cmake . -DBUILD_SHARED_LIBS=ON
make
sudo make install
sudo ldconfig
cd -
