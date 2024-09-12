#ifndef _MESHLIB_PARSER_H_
#define _MESHLIB_PARSER_H_

#include <string>
#include <assert.h>
#include <list>

namespace MeshLib
{

class CToken
{
public:
	std::string m_key;
	std::string m_value;
};


class CParser
{
public:

	CParser( std::string & str);
	~CParser();

  std::list<CToken*> & tokens() { return m_tokens; };

private:

	char next_char();
	void skip_blank();
	bool end();

  std::list<CToken*> m_tokens;

	char m_line[1024];
	char m_key[1024];
	char m_value[1024];

	char * m_pt;
};


}; //namespace 
#endif