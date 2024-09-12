#include "Parser.h"

using namespace MeshLib;

CParser::~CParser()
{
  for( std::list<CToken*>::iterator titer = m_tokens.begin(); titer != m_tokens.end(); titer ++ )
  {
      CToken * pT = *titer;
      delete pT;
  }
  m_tokens.clear();
};

CParser::CParser( std::string & str )
{

	//copy string
	unsigned int i;
	for(i = 0; i < str.length(); i ++ )
	{
		m_line[i] = str.c_str()[i];
	}
	m_line[i] = 0;

	m_pt = m_line;

	while( !end() )
	{
		skip_blank();//skip后m_pt指针所指位置非空
		if( end() ) break;

		//copy key

		char * pkey = m_key;
		char   ch = next_char();//注意ch仍为m_pt原始位置，m_pt则已经后移一位

		while( ch != ' ' && ch != '=' && !end() )
		{
			*pkey ++ = ch;
			ch = next_char();
		}
		if( ch != '=' && ch != ' ')//对应m_pt指向\0时的情况，此时ch所指向位置尚未录入
			*pkey++ = ch;

		*pkey = 0;

		if( ch == ' ' )
		{
			skip_blank();
		}

		if( ch != '=' )//说明该key没有对应的value，故可以直接录入token
		{
			CToken *	tk = new CToken;
			assert(tk);
			tk->m_key  = std::string( m_key );
			
			m_tokens.push_back( tk );
			continue;
		}

		if( end() ) break;

		ch = next_char();

		while( ch != '(' ) ch = next_char();

		char * pvalue = m_value;

		while( ch != ')' && !end() )
		{
			*pvalue ++ = ch;
			ch = next_char();
		}
		*pvalue++ = ch;
		*pvalue  = 0;//pvalue把括号也录进去了

		CToken *	tk = new CToken;
		assert(tk);
		tk->m_key   = std::string( m_key   );
		tk->m_value = std::string( m_value );
		
		m_tokens.push_back( tk );
		
	}

};

char CParser::next_char()
{
	char ch= *m_pt;
	m_pt++;
	return ch;
};

void CParser::skip_blank()
{
	while( *m_pt == ' ' ) m_pt++;
};

bool CParser::end()
{
	return ( (*m_pt) == 0 );
};

