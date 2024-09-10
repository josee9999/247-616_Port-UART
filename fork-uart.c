#define _GNU_SOURCE

#include <stdio.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include <unistd.h>  // UNIX Standard Definitions
#include <errno.h>   // ERROR Number Definitions

const char *portTTY = "/dev/ttyS1";
int fd; // File Descriptor

void main(void){
     
    InitPortSerie();

    int pipefd[2];
    pid_t pid;
    char buf;

    // Créer le pipe
    if (pipe(pipefd) == -1) {
        perror("pipe");
        return -1;
    }

    // Créer un processus enfant
    pid = fork();
    if (pid == -1) { // Une erreur s'est produite
        perror("fork");
        return -1;
    }

    if (pid == 0) { // Code exécuté par le processus enfant
        printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)...\n");

        // Write data to serial port 
	    char write_buffer[32];	    // Buffer containing characters to write into port
	    int  bytes_written  = 0;  	// Value for storing the number of bytes written to the port 
        char cpt = 0;
        char cCharLu;

        cCharLu=getchar();
        
        while(cCharLu != 'q'){
            write_buffer[cpt]=cCharLu;
            cpt++;
            cCharLu=getchar();
        }


	    bytes_written = write(fd, write_buffer, cpt); // use write() to send data to port 
										// "fd"                   - file descriptor pointing to the opened serial port
										// "write_buffer"         - address of the buffer containing data
										// "sizeof(write_buffer)" - No of bytes to write 
	    printf("\n Ecriture de %d octets : %s ecrit sur le port %s", cpt, write_buffer, portTTY);
	    printf("Fin du Fils\n");
    }

    else {  // Code exécuté par le processus parent
        printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série...\n");

        // Read data from serial port 
	    tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
	    char read_buffer[32];   // Buffer to store the data received 
	    int  bytes_read = 0;    // Number of bytes read by the read() system call 
	    int i = 0;

        while(read_buffer[i] != '!' && i<32){
	        bytes_read = read(fd, &read_buffer, 32); // Read the data 
            i++;
        }

	    printf("processus Père: nombre d'octets recus : %d --> ", bytes_read); // Print the number of bytes read
	    for(i=0; i<bytes_read; i++)	 // printing only the received characters
		    printf("%c", read_buffer[i]);
         

    	printf("Fin du Père\n");
        
    }
    close(fd); // Close the serial port
}

void InitPortSerie(void){
	
	printf("\n Initialisation du Port Serie");

	// Opening the Serial Port 
	fd = open(portTTY, O_RDWR | O_NOCTTY);  
							// O_RDWR   - Read/Write access to serial port 
							// O_NOCTTY - No terminal will control the process
							// Open in blocking mode,read will wait 
	if(fd == -1) // Error Checking
		printf("\n Erreur! ouverture de %s ", portTTY);
	else
		printf("\n Ouverture de %s reussit ", portTTY);

	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  
	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
	SerialPortSettings.c_cc[VMIN] = 1; // Read at least X character(s) 
	SerialPortSettings.c_cc[VTIME] = 30; // Wait 10sec (0 for indefinetly) 

	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
		printf("\n  Erreur! configuration des attributs du port serie");
		
}

