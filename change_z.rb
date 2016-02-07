# =========
# Utilities
# =========

# =========
# =========
def extract_data( filename, value )
  data = IO.readlines( filename )
  for i in 0..data.size()-1
    #Get rid of special trailing characters
    data[i].chomp!(',')
    #then, strip away the identifying characters at the front
    data[i] = data[i].split(' ')
    for j in (0..data[i].size()-1)
      #Cut out the leading white space
      data[i][j].strip!
      if j == 2
        data[i][j] = data[i][j].to_f
        data[i][j] = data[i][j] + value
      end
    end
  end
  return data
end


# ============
# Main Routine
# ============
def main()
    if ARGV.size < 2
      print "ERROR, not enough arguments"
    end

    #Alright, first, let's load up some files
    prefix = ARGV[0]
    value = ARGV[1]
    value = value.to_f

    # print "Value is: " << value.to_s << "\n"    

    data = []
    files = []

    #Move into the prefix directory
    Dir.chdir( prefix )
    #Get the file names
    files = Dir.glob("*")

    index = 0
    #Get all the filenames
    for file in files
      #And extract all the data
      data = (extract_data(file, value))
      #Show the mapping
      new_name = file + ".new"
      print file << " => " << new_name << "\n"
      
      fout = File.open(new_name, "w") 
      for i in 0 .. (data.size()-1)
        for j in 0 .. (data[i].size()-1)
          fout << data[i][j] << " "
        end
        fout << "\n"
      end
      fout.close
    end

end

main()
