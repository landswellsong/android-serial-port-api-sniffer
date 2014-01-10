/*
 * Copyright 2009-2011 Cedric Priscal
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <jni.h>

#include "SerialPort.h"

#include "android/log.h"
static const char *TAG="serial_port";
#define LOGI(fmt, args...) __android_log_print(ANDROID_LOG_INFO,  TAG, fmt, ##args)
#define LOGD(fmt, args...) __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define LOGE(fmt, args...) __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##args)

static speed_t getBaudrate(jint baudrate)
{
	switch(baudrate) {
	case 0: return B0;
	case 50: return B50;
	case 75: return B75;
	case 110: return B110;
	case 134: return B134;
	case 150: return B150;
	case 200: return B200;
	case 300: return B300;
	case 600: return B600;
	case 1200: return B1200;
	case 1800: return B1800;
	case 2400: return B2400;
	case 4800: return B4800;
	case 9600: return B9600;
	case 19200: return B19200;
	case 38400: return B38400;
	case 57600: return B57600;
	case 115200: return B115200;
	case 230400: return B230400;
	case 460800: return B460800;
	case 500000: return B500000;
	case 576000: return B576000;
	case 921600: return B921600;
	case 1000000: return B1000000;
	case 1152000: return B1152000;
	case 1500000: return B1500000;
	case 2000000: return B2000000;
	case 2500000: return B2500000;
	case 3000000: return B3000000;
	case 3500000: return B3500000;
	case 4000000: return B4000000;
	default: return -1;
	}
}

/* Hooks the requests to the device in path that has a fd descriptor */
struct snifferarg
{
	int realfd;
	int socket;
	time_t opentime;
	const char* devname;
};

/* TODO: error checks */
static void portsniffer(void *arg)
{
	/* Obtaining a local copy of task spec and freeing the params */
	struct snifferarg task;
	memcpy(&task, arg, sizeof(struct snifferarg));
	char *devname=((struct snifferarg *)arg)->devname;
	task.devname=malloc(strlen(devname));
	strcpy(task.devname, devname); /* TODO: this may leak on premature return, use goto */
	free(arg);
	
	/* Open the file descriptor, fill the header */
	char fname[256];
	sprintf(fname, "/tmp/%d.json", (long)task.opentime);
	FILE *fp=fopen(fname,"w");
	if (!fp)
	{
		LOGE("Cannot open %s for writing.", fname);
		return;
	}
	fprintf(fp,"{ \"sniffer\" : { \"devname\" : \"%s\", \"opentime\" : %d, \"measurements\" : [\n", task.devname, (long)task.opentime);
	
	/* Waiting for the events */
	int fds[2] = { task.realfd, task.socket }; /* TODO: check if closing the socket triggers anything, if not, use poll() */
	const char *dirs[2] = { "in", "out" };
	int i;
	for (;;)
	{
		fd_set rfds,efds;
		FD_ZERO(&rfds); FD_ZERO(&efds);
		for (i=0;i<2;i++)
		{
			FD_SET(fds[i],&rfds);
			FD_SET(fds[i],&efds);
		}
		int r;
		if ( ( r = select(max(fds[0],fds[1]) + 1, &rfds, NULL, &efds, NULL) ) != -1 )
		{
			LOGE("select() failed");
			return;
		}
			
		/* First, check for errors */
		if (FD_ISSET(fds[0],efds) || FD_ISSET(fds[1],efds))
		{
			close(task.socket); close(task.realfd);
			break;
		}
		
		/* Great, let's see how much data we have to read */
		for (i=0;i<2;i++)
		{
			if (FD_ISSET(fds[i]),rfds)
			{
				int navail = -1;
				if (ioctl(fds[i], FIONREAD, &navail) < 0)
				{
					LOGE("ioctl() failed");
					return;
				}
				
				/* Now we know the amount, read the data and pass it on */
				char *buf=malloc(navail);
				read(fds[i], buf, navail);
				write(fds[!i], buf, navail);
				
				/* Log the whole thing TODO: should we log the data "as is" or in hexes? */
				fprintf(fp,"{ \"direction\" : \"%s\", \"time\" : %d, \"data\" : \"%s\" }\n", dirs[i], mktime(localtime(NULL)), buf);
				
				free(buf);
			}
		}
	}
	
	/* Okay, the other of the socket is closed, let's finish */
	fprintf(fp,"] } }\n");
	fclose(fp);
	free(task.devname);
}

/*
 * Class:     android_serialport_SerialPort
 * Method:    open
 * Signature: (Ljava/lang/String;II)Ljava/io/FileDescriptor;
 */
JNIEXPORT jobject JNICALL Java_android_1serialport_1api_SerialPort_open
  (JNIEnv *env, jclass thiz, jstring path, jint baudrate, jint flags)
{
	int fd;
	int sockets[2];
	speed_t speed;
	jobject mFileDescriptor;
	const char *path_utf;

	/* Check arguments */
	{
		speed = getBaudrate(baudrate);
		if (speed == -1) {
			/* TODO: throw an exception */
			LOGE("Invalid baudrate");
			return NULL;
		}
	}

	/* Opening device */
	{
		jboolean iscopy;
		path_utf = (*env)->GetStringUTFChars(env, path, &iscopy);
		LOGD("Opening serial port %s with flags 0x%x", path_utf, O_RDWR | flags);
		fd = open(path_utf, O_RDWR | flags);
		LOGD("open() fd = %d", fd);

		if (fd == -1)
		{
			/* Throw an exception */
			LOGE("Cannot open port");
			/* TODO: throw an exception */
			return NULL;
		}
	}

	/* Configure device */
	{
		struct termios cfg;
		LOGD("Configuring serial port");
		if (tcgetattr(fd, &cfg))
		{
			LOGE("tcgetattr() failed");
			close(fd);
			/* TODO: throw an exception */
			return NULL;
		}

		cfmakeraw(&cfg);
		cfsetispeed(&cfg, speed);
		cfsetospeed(&cfg, speed);

		if (tcsetattr(fd, TCSANOW, &cfg))
		{
			LOGE("tcsetattr() failed");
			close(fd);
			/* TODO: throw an exception */
			return NULL;
		}
	}

	/* Create a socket pair and launch the thread */
	{
		if (socketpair(AF_LOCAL, SOCK_STREAM, 0, sockets) < 0)
		{
			LOGE("socketpair() failed");
			return;
		}
		
		struct snifferarg *args = (struct snifferargs *)malloc(sizeof(struct snifferargs));
		args->realfd=fd;
		args->socket=sockets[0];
		args->opentime=mktime(localtime(NULL));
		args->devname=path_utf;
		
		pthread_t threadid;
		/* TODO: study attributes */
		if (pthread_create(&threadid, NULL, portsniffer, (void* )args);
	}

	/* Create a corresponding file descriptor */
	{
		jclass cFileDescriptor = (*env)->FindClass(env, "java/io/FileDescriptor");
		jmethodID iFileDescriptor = (*env)->GetMethodID(env, cFileDescriptor, "<init>", "()V");
		jfieldID descriptorID = (*env)->GetFieldID(env, cFileDescriptor, "descriptor", "I");
		mFileDescriptor = (*env)->NewObject(env, cFileDescriptor, iFileDescriptor);
		(*env)->SetIntField(env, mFileDescriptor, descriptorID, (jint)socket[1]);
		(*env)->ReleaseStringUTFChars(env, path, path_utf); /* TODO: possible race condition with the thread, need a mutex */
	}

	return mFileDescriptor;
}

/*
 * Class:     cedric_serial_SerialPort
 * Method:    close
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_android_1serialport_1api_SerialPort_close
  (JNIEnv *env, jobject thiz)
{
	jclass SerialPortClass = (*env)->GetObjectClass(env, thiz);
	jclass FileDescriptorClass = (*env)->FindClass(env, "java/io/FileDescriptor");

	jfieldID mFdID = (*env)->GetFieldID(env, SerialPortClass, "mFd", "Ljava/io/FileDescriptor;");
	jfieldID descriptorID = (*env)->GetFieldID(env, FileDescriptorClass, "descriptor", "I");

	jobject mFd = (*env)->GetObjectField(env, thiz, mFdID);
	jint descriptor = (*env)->GetIntField(env, mFd, descriptorID);

	LOGD("close(fd = %d)", descriptor);
	close(descriptor);
}

