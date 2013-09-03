#!/bin/bash
ls $* | sed 's/^/- /g' > list.yaml
