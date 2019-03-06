#!/usr/bin/make -f

dirs = Pinocchio AttachWeights DemoUI

# Define a standard makePerDir command, which goes into
# each dir, and runs make $(makerule)
define makePerDir
for dir in $(dirs); \
do \
	cd $$dir && { $(MAKE) $(makerule); cd ..; }; \
done
endef

nullstring :=

all: makerule = $(nullstring)
depend: makerule = depend
clean: makerule = clean
distclean: makerule = distclean

all depend clean distclean:
	$(makePerDir)

.PHONY: all depend clean distclean
