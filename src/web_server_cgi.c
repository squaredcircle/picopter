// Web server CGI code to handle data from web browser and provide feedback
// Chris Venables
// 26/10/2013
// christopher.venables@uwa.edu.au

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static char logstr [1024];

int log_file(char* log_string)	//function to add info to log file
{
FILE *fp;    /* File pointer */
   /* Open the log file for writing */
   if (NULL == (fp = fopen("/home/pi/Web_Log_Files/pid3.txt","a"))) {
      printf("Error printing to text log file\n"); // if writing fails print error and exit
      return 1;
   }
   
char print_log_str[1024];
sprintf(print_log_str,"%s",log_string);
   fprintf(fp, print_log_str);  /* write the provided string to the file */
   
   if (!0 == fclose(fp)){ /* close the file we opened earlier*/
	printf("Error closing text log file\n");
	return 1;
   }
return 0;
}

int create_log_file(void)
{
	//Create Log File
	printf("\nCreate web log file\n");
FILE *fp;    /* File pointer */
if(NULL != (fp = fopen("/home/pi/Web_Log_Files/pid3.txt", "w")))
{
	system("chmod 777 /home/pi/Web_Log_Files/pid3.txt");
	printf("Updated file: /home/pi/Web_Log_Files/pid3.txt\n");
}
else
{
printf("Error creating output data file\n");
}
fclose(fp);

return 0;
}

int main(void)
{
char *data;
double Kpx,Kpy,Ki,Kd,campitch;
printf("<html>%s%c%c\n",
"Content-Type:text/html;charset=iso-8859-1",13,10);
printf("<head><TITLE>PID Data</TITLE></head>\n");
printf("<body><H3>PID Data</H3>\n");
data = getenv("QUERY_STRING");
if(data == NULL)
  printf("<P>Error! Error in passing data from form to script.");
else if(sscanf(data,"Kpx=%lf&Kpy=%lf&Ki=%lf&Kd=%lf&campitch=%lf",&Kpx,&Kpy,&Ki,&Kd,&campitch)!=5)
  printf("<P>Error! Invalid data. Data must be numeric.");
else
  printf("<P>The chosen PID data is Kpx = %lf Kpy = %lf Ki = %lf Kd = %lf and campitch = %lf.",Kpx,Kpy,Ki,Kd,campitch);
  
create_log_file();

sprintf(logstr,"%lf\n",Kpx);
log_file(logstr);

sprintf(logstr,"%lf\n",Kpy);
log_file(logstr);

sprintf(logstr,"%lf\n",Ki);
log_file(logstr);
  
sprintf(logstr,"%lf\n",Kd);
log_file(logstr);  
    
sprintf(logstr,"%lf\n",campitch);
log_file(logstr);  
  
printf("</body></html>");
return 0;
}