#line 1 "..\\USMART\\usmart_str.c"
#line 1 "..\\USMART\\usmart_str.h"





















































typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;
						  
u8 usmart_get_parmpos(u8 num);						
u8 usmart_strcmp(u8*str1,u8 *str2);					
u32 usmart_pow(u8 m,u8 n);							
u8 usmart_str2num(u8*str,u32 *res);					
u8 usmart_get_cmdname(u8*str,u8*cmdname,u8 *nlen,u8 maxlen);
u8 usmart_get_fname(u8*str,u8*fname,u8 *pnum,u8 *rval);		
u8 usmart_get_aparm(u8 *str,u8 *fparm,u8 *ptype); 	
u8 usmart_get_fparam(u8*str,u8 *parn);  			












#line 2 "..\\USMART\\usmart_str.c"
#line 1 "..\\USMART\\usmart.h"
#line 4 "..\\USMART\\usmart.h"












































































 
struct _m_usmart_nametab
{
	void* func;			
	const u8* name;		
};

struct _m_usmart_dev
{
	struct _m_usmart_nametab *funs;	

	void (*init)(u8);				
	u8 (*cmd_rec)(u8*str);			
	void (*exe)(void); 				
	void (*scan)(void);             
	u8 fnum; 				  		
	u8 pnum;                        
	u8 id;							
	u8 sptype;						
	u16 parmtype;					
	u8  plentbl[10];  		
	u8  parm[200];  			
};
extern struct _m_usmart_nametab usmart_nametab[];	
extern struct _m_usmart_dev usmart_dev;				


void usmart_init(u8 sysclk);
u8 usmart_cmd_rec(u8*str);	
void usmart_exe(void);		
void usmart_scan(void);     
u32 read_addr(u32 addr);	
void write_addr(u32 addr,u32 val);































#line 3 "..\\USMART\\usmart_str.c"



















































  




u8 usmart_strcmp(u8 *str1,u8 *str2)
{
	while(1)
	{
		if(*str1!=*str2)return 1;
		if(*str1=='\0')break;
		str1++;
		str2++;
	}
	return 0;
}



void usmart_strcopy(u8*str1,u8 *str2)
{
	while(1)
	{										   
		*str2=*str1;	
		if(*str1=='\0')break;
		str1++;
		str2++;
	}
}



u8 usmart_strlen(u8*str)
{
	u8 len=0;
	while(1)
	{							 
		if(*str=='\0')break;
		len++;
		str++;
	}
	return len;
}


u32 usmart_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}	    







u8 usmart_str2num(u8*str,u32 *res)
{
	u32 t;
	u8 bnum=0;	
	u8 *p;		  
	u8 hexdec=10;
	p=str;
	*res=0;
	while(1)
	{
		if((*p<='9'&&*p>='0')||(*p<='F'&&*p>='A')||(*p=='X'&&bnum==1))
		{
			if(*p>='A')hexdec=16;	
			bnum++;					
		}else if(*p=='\0')break;	
		else return 1;				
		p++; 
	} 
	p=str;			    
	if(hexdec==16)		
	{
		if(bnum<3)return 2;			
		if(*p=='0' && (*(p+1)=='X'))
		{
			p+=2;	
			bnum-=2;
		}else return 3;
	}else if(bnum==0)return 4;
	while(1)
	{
		if(bnum)bnum--;
		if(*p<='9'&&*p>='0')t=*p-'0';	
		else t=*p-'A'+10;				
		*res+=t*usmart_pow(hexdec,bnum);		   
		p++;
		if(*p=='\0')break;
	}
	return 0;
}






u8 usmart_get_cmdname(u8*str,u8*cmdname,u8 *nlen,u8 maxlen)
{
	*nlen=0;
 	while(*str!=' '&&*str!='\0') 
	{
		*cmdname=*str;
		str++;
		cmdname++;
		(*nlen)++;
		if(*nlen>=maxlen)return 1;
	}
	*cmdname='\0';
	return 0;
}



u8 usmart_search_nextc(u8* str)
{		   	 	
	str++;
	while(*str==' '&&str!='\0')str++;
	return *str;
} 






u8 usmart_get_fname(u8*str,u8*fname,u8 *pnum,u8 *rval)
{
	u8 res;
	u8 fover=0;	  
	u8 *strtemp;
	u8 offset=0;  
	u8 parmnum=0;
	u8 temp=1;
	u8 fpname[6];
	u8 fplcnt=0; 
	u8 pcnt=0;	 
	u8 nchar;
	
	strtemp=str;
	while(*strtemp!='\0')
	{
		if(*strtemp!=' '&&(pcnt&0X7F)<5)
		{	
			if(pcnt==0)pcnt|=0X80;
			if(((pcnt&0x7f)==4)&&(*strtemp!='*'))break;
			fpname[pcnt&0x7f]=*strtemp;
			pcnt++;
		}else if(pcnt==0X85)break;
		strtemp++; 
	} 
	if(pcnt)
	{
		fpname[pcnt&0x7f]='\0';
		if(usmart_strcmp(fpname,"void")==0)*rval=0;
		else *rval=1;							   
		pcnt=0;
	} 
	res=0;
	strtemp=str;
	while(*strtemp!='('&&*strtemp!='\0') 
	{  
		strtemp++;
		res++;
		if(*strtemp==' '||*strtemp=='*')
		{
			nchar=usmart_search_nextc(strtemp);		
			if(nchar!='('&&nchar!='*')offset=res;	
		}
	}	 
	strtemp=str;
	if(offset)strtemp+=offset+1;
	res=0;
	nchar=0;
	while(1)
	{
		if(*strtemp==0)
		{
			res=1;
			break;
		}else if(*strtemp=='('&&nchar==0)fover++;
		else if(*strtemp==')'&&nchar==0)
		{
			if(fover)fover--;
			else res=1;
			if(fover==0)break;
		}else if(*strtemp=='"')nchar=!nchar;

		if(fover==0)
		{
			if(*strtemp!=' ')
			{
				*fname=*strtemp;
				fname++;
			}
		}else 
		{
			if(*strtemp==',')
			{
				temp=1;		
				pcnt++;	
			}else if(*strtemp!=' '&&*strtemp!='(')
			{
				if(pcnt==0&&fplcnt<5)		
				{
					fpname[fplcnt]=*strtemp;
					fplcnt++;
				}
				temp++;	
			}
			if(fover==1&&temp==2)
			{
				temp++;		
				parmnum++; 	
			}
		}
		strtemp++; 			
	}   
	if(parmnum==1)
	{
		fpname[fplcnt]='\0';
		if(usmart_strcmp(fpname,"void")==0)parmnum=0;
	}
	*pnum=parmnum;	
	*fname='\0';	
	return res;		
}







u8 usmart_get_aparm(u8 *str,u8 *fparm,u8 *ptype)
{
	u8 i=0;
	u8 enout=0;
	u8 type=0;
	u8 string=0; 
	while(1)
	{		    
		if(*str==','&& string==0)enout=1;			
		if((*str==')'||*str=='\0')&&string==0)break;
		if(type==0)
		{
			if((*str>='0' && *str<='9')||(*str>='a' && *str<='f')||(*str>='A' && *str<='F')||*str=='X'||*str=='x')
			{
				if(enout)break;					
				if(*str>='a')*fparm=*str-0X20;	
				else *fparm=*str;		   		
				fparm++;
			}else if(*str=='"')
			{
				if(enout)break;
				type=1;
				string=1;
			}else if(*str!=' '&&*str!=',')
			{
				type=0XFF;
				break;
			}
		}else
		{
			if(*str=='"')string=0;
			if(enout)break;					
			if(string)					    
			{	 
				*fparm=*str;		   		
				fparm++;
			}	
		}
		i++;
		str++;
	}
	*fparm='\0';	
	*ptype=type;	
	return i;		
}



u8 usmart_get_parmpos(u8 num)
{
	u8 temp=0;
	u8 i;
	for(i=0;i<num;i++)temp+=usmart_dev.plentbl[i];
	return temp;
}




u8 usmart_get_fparam(u8*str,u8 *parn)
{	
	u8 i,type;  
	u32 res;
	u8 n=0;
	u8 len;
	u8 tstr[200+1];
	for(i=0;i<10;i++)usmart_dev.plentbl[i]=0;
	while(*str!='(')
	{
		str++;											    
		if(*str=='\0')return 1;
	}
	str++;
	while(1)
	{
		i=usmart_get_aparm(str,tstr,&type);	
		str+=i;								
		switch(type)
		{
			case 0:	
				if(tstr[0]!='\0')				
				{					    
					i=usmart_str2num(tstr,&res);	
					if(i)return 2;		
					*(u32*)(usmart_dev.parm+usmart_get_parmpos(n))=res;
					usmart_dev.parmtype&=~(1<<n);	
					usmart_dev.plentbl[n]=4;		
					n++;							
					if(n>10)return 3;
				}
				break;
			case 1:
				len=usmart_strlen(tstr)+1;	
				usmart_strcopy(tstr,&usmart_dev.parm[usmart_get_parmpos(n)]);
				usmart_dev.parmtype|=1<<n;	
				usmart_dev.plentbl[n]=len;	
				n++;
				if(n>10)return 3;
				break;
			case 0XFF:
				return 2;
		}
		if(*str==')'||*str=='\0')break;
	}
	*parn=n;	
	return 0;
}














