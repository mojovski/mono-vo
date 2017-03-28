#ifndef CSV_READER_H
#define CSV_READER_H


#include <map>
#include <stdexcept>
#include <fstream>

class CSVReader
{
public:
	class CSVRow{
	public:
		std::vector<std::string> elements;
	};
	typedef std::vector<CSVRow> LinesType;

	CSVReader():separator(","){};
	std::string separator;
	

	LinesType lines;


	void read(std::string path)
		{
			std::ifstream  file(path.c_str());
			std::string line;
			if (file.is_open())
			{
				while ( getline (file,line) )
				{
					CSVRow csv_line=parseLine(line);
					lines.push_back(csv_line);

				}
			} else
			{
				std::cout << "\nUnable to open file " << path;
				throw std::runtime_error("Cannot open the file "+path);
			}
		}

	protected:
		CSVRow parseLine(std::string& line)
		{
			std::string cache;
			std::vector<std::string> tokens;
			for (int i=0; i<line.size(); i++)
			{
				if (line.at(i) == ',')
				{
					if (cache!="")
						tokens.push_back(cache);
					cache="";
				} else
				{
					//std::cout << "char: " << fields_str.at(i) << std::endl;
					cache+=line.at(i);
				}
			}
			cache.erase( std::remove(cache.begin(), cache.end(), '\r'), cache.end() );
			tokens.push_back(cache);
			//now fill the row
			CSVRow row;
			for(std::vector<std::string>::iterator it=tokens.begin(); it!=tokens.end();++it)
			{
				row.elements.push_back(*it);
		   	}
			return row;
		}


};




#endif