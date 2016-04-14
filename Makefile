PREFIX ?= usr/local

sterm: main.c Makefile
	gcc -O2 -Wall -Werror -o $@ $<

install:
	install -d -m 0755 $(DESTDIR)/$(PREFIX)/bin
	install -m 0755 sterm $(DESTDIR)/$(PREFIX)/bin
