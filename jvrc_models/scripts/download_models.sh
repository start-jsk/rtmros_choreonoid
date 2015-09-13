#!/bin/sh

mkdir -p `rospack find jvrc_models`/models
scp ${SSH_USER}@aries:/home/jsk/ohara/jvrc_models/*  `rospack find jvrc_models`/models
