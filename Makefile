#To build...
#make -C /home/pi/linux M=$PWD
#(if your linux source tree lives at /home/pi/linux, for instance)

obj-m:=snd_pcm_pi_i2s.o

clean:
	rm -f *.ko
	rm -f *.o
	rm -f *.mod.c
	rm -f Module.symvers
	rm -f modules.order
	rm -fr .tmp_versions
	rm -fr .built-in.o.cmd
	rm -fr .snd_pcm_pi_i2s*
	rm -fr .gout*     
