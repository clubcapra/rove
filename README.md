# Rove

## Native installation
```bash
mkdir -p ~/capra/src
cd ~/capra
wget https://raw.githubusercontent.com/clubcapra/rove/master/rove.repos
vcs import src < rove.repos
```




## Docker architecture

It's possible to run the entire project into multiple docker containers. Each container can be run independantly and are built using the following structure : 

![Docker structure](https://www.plantuml.com/plantuml/svg/VP6zaeGm2CTxdo9Zxrn_nSqMsznJt63aTZERZqmWFlz573UU1ON27twWm8qO2jVW1tRiqOptP5zOp7U01vexPemBHkkGnc4eQ1dYOyDAee_sV3vhc3rE2zABKnuDa6dXCsbzdIta0erVyMS6Gi4sH-5SP2o_O964xbBhZKzONIfISGY5Nnsv58NUNOM5oYccu53mjr8go8NgWOylTAdKC9F0pHgjDRDWpIfKz5MePWS5orWivlT_TjcA4fbf-jfljRr4dMxHNSdiMxn4-x8kYNwYmv4eC_qBo9JdW0nqQTMtUycSvxbXN6hmVm00 "Docker structure")

# Documentation

To update the UML diagram, you need to create a new encoded link using the plantuml website, copy the current UML and change the markdown link.