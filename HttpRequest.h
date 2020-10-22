
/*
 *  © 2012 Francisco G. Paletta, © 2020 Gregor Baues. All rights reserved.
 *  
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HttpRequest_h
#define HttpRequest_h

#include "Ethernet.h"

// Buffer lengths
#define HTTP_REQ_METHOD_LENGTH 			10		//10 is enough
#define HTTP_REQ_URI_LENGTH 			32		//adjust if you have long path/file names
#define HTTP_REQ_VERSION_LENGTH 		10		//10 is enough
#define HTTP_REQ_PARAM_NAME_LENGTH 		16		//adjust to meet your needs
#define HTTP_REQ_PARAM_VALUE_LENGTH 	16		//adjust to meet your needs
#define HTTP_REQ_ATTRIB_NAME_LENGTH 	16		//enough to track attribute name
#define HTTP_REQ_ATTRIB_VALUE_LENGTH 	16 		//enough to track "Content-Length" value
#define HTTP_REQ_COOKIE_NAME_LENGTH 	10		//adjust to meet your needs
#define HTTP_REQ_COOKIE_VALUE_LENGTH 	16 		//adjust to meet your needs

// Parsing status
#define HTTP_PARSE_INIT		0		//Initial Parser Status
#define HTTP_METHOD 		0		//Parse the Method: GET POST UPDATE etc
#define HTTP_URI			1		//Parse the URI
#define HTTP_GET_NAME		11		//Parse the GET parameter NAME
#define HTTP_GET_VALUE		12		//Parse the GET parameter VALUE
#define HTTP_VERSION 		2		//Parse the version: HTTP1.1
#define HTTP_NEW_LINE		3		//Starts reading a new line
#define HTTP_ATTRIB_NAME	41		//Read the attibutes NAME
#define HTTP_ATTRIB_VALUE	42		//Read the attribute VALUE
#define HTTP_POST_NAME		51		//Read the POST parameter NAME
#define HTTP_POST_VALUE		52		//Read the POST paramenter VALUE
#define HTTP_COOKIE_NAME	61		//Read the COOKIE NAME
#define HTTP_COOKIE_VALUE	62		//Read the COOKIE VALUE
#define HTTP_REQUEST_END	99		//Finished reading the HTTP Request

// returned to the callback
struct ParsedRequest {
	char* method;
	char* uri;
	char* version;
	uint8_t* paramCount;
	// uint8_t (*getByIndex)(int, char*, char*);
	// uint8_t (*getByName)(char*, char*);
};

	struct Params
	{
		char name[HTTP_REQ_PARAM_NAME_LENGTH];
		char value[HTTP_REQ_PARAM_VALUE_LENGTH];
		Params *next;
	};

class HttpRequest
{
private:

	// no Cookies
	struct Cookies
	{
		char name[HTTP_REQ_COOKIE_NAME_LENGTH];
		char value[HTTP_REQ_COOKIE_VALUE_LENGTH];
		Cookies *next;
	};

	uint8_t parseStatus;
	Params *firstParam;								
	Cookies *firstCookie;

	char tmpParamName[HTTP_REQ_PARAM_NAME_LENGTH];
	char tmpParamValue[HTTP_REQ_PARAM_VALUE_LENGTH];
	char tmpAttribName[HTTP_REQ_ATTRIB_NAME_LENGTH];
	char tmpAttribValue[HTTP_REQ_ATTRIB_NAME_LENGTH];
	char tmpCookieName[HTTP_REQ_COOKIE_NAME_LENGTH]; // no use
	char tmpCookieValue[HTTP_REQ_COOKIE_VALUE_LENGTH]; // no use

	uint16_t dataBlockLength, dataCount;

	void addParam();
	void addAttrib();
	void addCookie();								// no use
	void freeParamMem(Params *paramNode);
	void freeCookieMem(Cookies *cookieNode);

	char method[HTTP_REQ_METHOD_LENGTH];				// user
	char uri[HTTP_REQ_URI_LENGTH];						// user
	char version[HTTP_REQ_VERSION_LENGTH];				// user

	uint8_t paramCount;										// user
	uint8_t cookieCount;								    // no use - no cookie support

	uint8_t getParam(uint8_t paramNum, char *name, char *value);  // user
	uint8_t getParam(char *name, char *value);				  // user
	uint8_t getCookie(uint8_t cookieNum, char *name, char *value); // no use
	uint8_t getCookie(char *name, char *value);				   // no use 

	ParsedRequest req;

public:
	HttpRequest();
	void resetRequest();
	void parseRequest(char c);
	bool endOfRequest();
	ParsedRequest getParsedRequest();
	Params* getParam(uint8_t paramNum); 
	void (* callback)(ParsedRequest *req, Client *client);
};

#endif
