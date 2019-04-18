#ifndef NMEA_H
#define NMEA_H
#include <Arduino.h>
#include <stdlib.h> // strtol(), 
#include "print.h"

class NMEA
{
public:
	String s; // $GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
	String type; // "GPRMC"
	double time; // 225446  (Time of fix 22:54:46 UTC)
	char warning; // A  (Navigation receiver warning A = OK, V = warning)
	double latitude; // 4916.45  (Latitude 49 deg. 16.45 min)
	char latitude_direction; // N  (Either N = North or S = South of the equator)
	double longitude; // 12311.12  (Longitude 123 deg. 11.12 min)
	char longitude_direction; // W  (Either E = East or W = West of the prime meridian)
	double smg; // 000.5  (Speed Made Good; speed over ground in Knots)
	double cmg; // 054.7  (Course Made Good; direction traveled from one fix to another, measured as angle from True North, positive clockwise degrees
	long date; // 191194 (Date of fix, Day Month Year,  19 November 1994)
	float magnetic_variation; // 020.3  (Magnetic Variation in degrees East or West)
	char magnetic_variation_direction; // E  (Direction of Magnetic Variation, either E = East or W = West)
	String provided_chk_hex; // 68  (Mandatory checksum, 0x68 (HEX, base16) == 104 (DEC, base10))
	int provided_chk; // 104  (Mandatory checksum in DEC, base10)

	bool valid; //  (this flag is false if the raw NMEA sentence does not start with '$' or end with '*XX')
	bool checksums_match; 
	String token; // holds 'words' from NMEA sentence. (Each chunk of data batween commas is a 'token')
	int start_index; // index of the first character of the token
	int end_index; // index of one position after the last character of the token 
	// ... for example: for the token 'word', start_token is index of 'w' and end_token is one index right of the 'd'
	



	// constructor - default
	NMEA()
	{
		this->s = "null";
		this->type = "null";
		this->time = -1.0;
		this->warning = '?';
		this->latitude = -1.0;
		this->latitude_direction = '?';
		this->longitude = -1.0;
		this->longitude_direction = '?';
		this->smg = -1.0;
		this->cmg = -1.0;
		this->date = -1;
		this->magnetic_variation = -1.0;
		this->magnetic_variation_direction = '?';
		this->provided_chk_hex = "null";
		this->provided_chk = -1;

		this->valid = false;
		this->checksums_match = false;
		this->token = "null";
		this->start_index = 0;
		this->end_index = -1; // -1 makes the next_token() function work	
	}

	// constructor - parameter
	NMEA(String s)
	{
		this->s = s;
		this->type = "null";
		this->time = -1.0;
		this->warning = '?';
		this->latitude = -1.0;
		this->latitude_direction = '?';
		this->longitude = -1.0;
		this->longitude_direction = '?';
		this->smg = -1.0;
		this->cmg = -1.0;
		this->date = -1;
		this->magnetic_variation = -1.0;
		this->magnetic_variation_direction = '?';
		this->provided_chk_hex = "null";
		this->provided_chk = -1;

		this->valid = false;
		this->checksums_match = false;
		this->token = "null";
		this->start_index = 0;
		this->end_index = -1; // -1 makes the next_token() function work	
	}


	// print (prints all member variables)
	void print_data(bool print_type=false)
	{
		if(print_type == false)
		{
			print(this->s);
			print(this->type);
			print(this->time);
			print(this->warning);
			print(this->latitude);
			print(this->latitude_direction);
			print(this->longitude);
			print(this->longitude_direction);
			print(this->smg);
			print(this->cmg);
			print(this->date);
			print(this->magnetic_variation);
			print(this->magnetic_variation_direction);
			print(this->provided_chk_hex);
			print(this->provided_chk);
			print(this->valid);
			print(this->checksums_match);
















			// TODO: finish adding member variables
























		}
	}





	// check_validity (checks to make sure the NMEA sentence starts with '$' and ends with '*XX')
	bool check_validity()
	{
		// make sure sentence starts with '$'
		if(!s.charAt(0) == '$')
		{
			print("Error! Sentence does not start with '$'");
			this->valid = false;
			return false;
		}

		char last_char = '?'; //s.charAt(s.length());
		int last_char_int = (int) last_char;

		// make sure sentence ends with '*XX'
		if( (s.length() - s.indexOf('*')) != 4) // changed from 3!!! (nmea sentence ends in NUL character '\0'.  '\r' gets removed int GPS.h)
		{
			print("Error! Sentence does not end with '*XX'");
			this->valid = false;
			return false;
		}
		this->valid = true;
		return true; 
	}



	// checksum (calculates the checksum and compares it with the checksum sent from the satalite)
	bool checksum()
	{
		// make sure sentence starts with '$'
		if(!s.startsWith("$"))
		{
			print("Error! Sentence does not start with '$'");
			return false;
		}

		// make sure sentence ends with '*XX'
		if( (s.length() - s.indexOf('*')) != 4) // changed from 3!!! (nmea sentence ends in NUL character '\0'.  '\r' gets removed int GPS.h)
		{
			print("Error! Sentence does not end with '*XX'");
			return false;
		}

		// save the checksum that was provided at the end of the original NMEA sentence
		provided_chk_hex = s.substring(s.indexOf('*') + 1); // 60 (String)
		provided_chk = (int)strtol( &provided_chk_hex[0], NULL, 16); // 96 (int)
		

		// running XOR calculation
		int chk = 0;
		for(int i=1; i < s.indexOf('*'); i++)
		{
			//print(s[i], true);
			int num = (int) s[i];
			//print(num, true);
			chk = chk ^ num;
		}

		// compare the provided checksum with the calculated checksum
		if(provided_chk == chk)
		{
			//print("Checksums match");
			checksums_match = true;
			return true;
		}
		else 
		{
			//print("Checksums do NOT match");
			checksums_match = false;
			return false;
		}
	}



	// next_token (advances start_index and end_index to the next token in the NMEA sentence)
	String next_token(char delim=',')
	{
		start_index = end_index + 1;
		end_index = s.indexOf(delim, start_index); // returns -1 if not found
		if(end_index == -1)
		{
			return "null";
		}
		
		token = s.substring(start_index, end_index);
		return token;
	}




	// parse_gprmc
	bool parse_gprmc()
	{	
		// skip storing type (it must be stored manually in main loop for parse() to work)
		next_token(); 

		// store time
		next_token(); // 225446
		if(!token.length() == 0) 
		{
			//time = token.toInt();
			time = token.toDouble();
		}
		else
		{
			time = -1.0; // could not determine time
		}
		


		// store warning
		next_token(); // A
		if(!token.length() == 0) 
		{
			warning = token.charAt(0);
		}
		else
		{
			warning = '?'; // could not determine warning
		}
		

		// store latitude
		next_token(); // 4916.45
		if(!token.length() == 0)
		{
			latitude = token.toDouble();
		}
		else
		{
			latitude = -1.0;
		}
		

		// store latitude_direction
		next_token(); // N
		if(!token.length() == 0)
		{
			latitude_direction = token.charAt(0);
		}
		else
		{
			latitude_direction = '?';
		}
		

		// store longitude
		next_token(); // 12311.12
		if(!token.length() == 0)
		{
			longitude = token.toDouble();
		}
		else
		{
			longitude = -1.0;
		}
		

		// store longitude_direction
		next_token(); // W
		if(!token.length() == 0)
		{
			longitude_direction = token.charAt(0);
		}
		else
		{
			longitude_direction = '?';
		}
		

		// store smg
		next_token(); // 000.5
		if(!token.length() == 0)
		{
			smg = token.toDouble();
		}
		else
		{
			smg = -1.0;
		}
		

		// store cmg
		next_token(); // 054.7
		if(!token.length() == 0)
		{
			cmg = token.toDouble();
		}
		else
		{
			cmg = -1.0;
		}
		


		// store date (TODO: not working correctly)
		next_token(); // 191194
		if(!token.length() == 0)
		{
			date = token.toInt();
		}
		else
		{
			date = -1;
		}



		// store magnetic_variation
		next_token(); // 020.3
		if(!token.length() == 0)
		{
			magnetic_variation = token.toFloat();
		}
		else
		{
			magnetic_variation = -1.0;
		}



		// store magnetic_variation_direction
		next_token('*'); // E
		if(!token.length() == 0)
		{
			magnetic_variation_direction = token.charAt(token.length()-1);
		}
		else
		{
			magnetic_variation_direction = '?';
		}
		

		// NOTE: provided_chk is stored every time the checksum() method is called


		// reset token and indeces
		token = "null";
		start_index = 0;
		end_index = -1;
		return true;
	}


	// parse (fill member variables with appropriate data from raw nmea sentence)
	bool parse()
	{
		if(this->type == "GPRMC")
		{
			//print("parse() function called");
			bool ret = parse_gprmc();
			if(ret)
			{
				return true;
			}
			else
			{
				return false;
			}
			
		}

		// TODO: add another case for GPGGA type nmea sentences
		// else if(this->type == "GPGGA")

		else
		{
			return false;
		}
	}


	// update
	bool update()
	{
		bool is_valid = false; 
		is_valid = check_validity();
		if(!is_valid)
		{
			print("Error! NMEA sentence not valid.");
			return false;
		}

		bool checksums_matched = false;
		checksums_matched = checksum();
		if(!checksums_matched)
		{
			print("Error! Checksums don't match.");
			return false;
		}

		//this->determine_type();

		bool parsed_successfully = false;
		parsed_successfully = this->parse();
		if(!parsed_successfully)
		{
			print("Error! parse unsuccessful.");
			return false;
		}

		return true;


	}

	
};
 
#endif



// REFERENCE: http://aprs.gids.nl/nmea/#rmc
