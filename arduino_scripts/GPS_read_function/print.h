#ifndef PRINT_H
#define PRINT_H

// string
void print(String var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: String)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}


// int
void print(int var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: int)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}


// unsigned int
void print(unsigned int var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: unsigned int)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}

// long
void print(long var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: long)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}


// unsigned long
void print(unsigned long var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: unsigned long)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}


// float
void print(float var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: float)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}


// double
void print(double var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: double)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}



// char
void print(char var, bool print_type=false)
{
	if(print_type)
	{
		Serial.print(var);
		Serial.flush();
		Serial.println(" (type: char)");
		Serial.flush();
	}
	else
	{
		Serial.println(var);
		Serial.flush();
	}
}


#endif

