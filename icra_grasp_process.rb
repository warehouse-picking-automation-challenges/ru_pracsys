# =========
# Utilities
# =========

TIME = 0
OBJECT = 1
ARM = 2
POSE = 3 
EVAL_T = 4
GRASP_T = 5
TOTAL_T = 6
TRIED = 7
SUCCESS = 8
REASON = 9

# =========
# =========
class String
def black;          "\e[30m#{self}\e[0m" end
def red;            "\e[31m#{self}\e[0m" end
def green;          "\e[32m#{self}\e[0m" end
def brown;          "\e[33m#{self}\e[0m" end
def blue;           "\e[34m#{self}\e[0m" end
def magenta;        "\e[35m#{self}\e[0m" end
def cyan;           "\e[36m#{self}\e[0m" end
def gray;           "\e[37m#{self}\e[0m" end
end
  
# =========
# =========
def extract_data( filename )
  data = IO.readlines( filename )
  for i in 0..data.size()-1
    #Get rid of special trailing characters
    data[i].chomp!(',')
    #then, strip away the identifying characters at the front
    data[i] = data[i].split(',')
    for j in 0..data[i].size()-1
      #Cut out the leading white space
      data[i][j].strip!
      if [4,5,6].include? j
        data[i][j] = data[i][j].to_f
      end
      if [3,7,8].include? j
        data[i][j] = data[i][j].to_i
      end
    end
  end
  return data
end

# =========
# =========
def get_slice( data_matrix, col )
    data = []
    for i in 0 .. (data_matrix.size - 1)
        idt = data_matrix[i]
        data.push( idt[col] )
    end
    return data
end

# =========
# =========
def average_data( dat )
  ret = sum_data( dat )
  ret /= dat.size
  return ret
end

# =========
# =========
def sum_data( dat )
  sum = 0
  for i in 0 .. (dat.size - 1)
    sum += dat[i]
  end
  return sum
end

# =========
# =========
def avg_for_arm_object( dat, arm, object )
  eval = grasp = total = tried = success = 0
  reasons = Hash.new
  count = 0.0
  
  for i in 0 .. dat.size-1
    if (dat[i][ARM] == arm) and (dat[i][OBJECT] == object)
      count = count + 1.0
      eval = eval + dat[i][EVAL_T]
      grasp = grasp + dat[i][GRASP_T]
      total = total + dat[i][TOTAL_T]
      tried = tried + dat[i][TRIED]
      success = success + dat[i][SUCCESS]
      if reasons[dat[i][REASON]] == nil
        reasons[dat[i][REASON]] = 1
      else
        reasons[dat[i][REASON]] = reasons[dat[i][REASON]] + 1
      end
    end
  end
  
  return [ eval/count, grasp/count, total/count, tried/count, success/count, reasons ]  
end

# =========
# =========
def avg_for_arm( dat, arm )
  eval = grasp = total = tried = success = 0
  reasons = Hash.new
  count = 0.0
  
  for i in 0 .. dat.size-1
    if (dat[i][ARM] == arm)
      count = count + 1.0
      eval = eval + dat[i][EVAL_T]
      grasp = grasp + dat[i][GRASP_T]
      total = total + dat[i][TOTAL_T]
      tried = tried + dat[i][TRIED]
      success = success + dat[i][SUCCESS]
      if reasons[dat[i][REASON]] == nil
        reasons[dat[i][REASON]] = 1
      else
        reasons[dat[i][REASON]] = reasons[dat[i][REASON]] + 1
      end
    end
  end
  
  return [ eval/count, grasp/count, total/count, tried/count, success/count, reasons ]  
end

# =========
# =========
def avg_for_object( dat, object )
  eval = grasp = total = tried = success = 0
  reasons = Hash.new
  count = 0.0
  
  for i in 0 .. dat.size-1
    if (dat[i][OBJECT] == object)
      count = count + 1.0
      eval = eval + dat[i][EVAL_T]
      grasp = grasp + dat[i][GRASP_T]
      total = total + dat[i][TOTAL_T]
      tried = tried + dat[i][TRIED]
      success = success + dat[i][SUCCESS]
      if reasons[dat[i][REASON]] == nil
        reasons[dat[i][REASON]] = 1
      else
        reasons[dat[i][REASON]] = reasons[dat[i][REASON]] + 1
      end
    end
  end
  
  return [ eval/count, grasp/count, total/count, tried/count, success/count, reasons ]  
end


# ============
# Main Routine
# ============
def main()
    #Alright, first, let's load up some files
    prefix = ARGV[0]
    data = []

    files = []

    #Move into the prefix directory
    Dir.chdir( prefix )
    #Get the file names
    files = Dir.glob("*")

    #Get all the filenames
    for file in files
        #And extract all the data
        data.push(extract_data(file))
    end
    #Peel away the outer layer
    data = data[0]

    done = false
    
    print "Welcome to the ICRA database, how may I help?\n\n".green
    while !done
      print "(e)xit, (a)rm, (o)bject, (b)oth:".brown
      command = STDIN.gets
      command.chomp!
      if command == 'e'
        print "Goodbye!\n"
        done = true
      elsif command == 'a'
        print "Which arm: "
        arm = STDIN.gets
        arm.chomp!
        print avg_for_arm( data, arm )        
        print "\n\n"
      elsif command == 'o'
        print "Which object: "
        object = STDIN.gets
        object.chomp!
        print avg_for_object( data, object )
        print "\n\n"         
      elsif command == 'b'
        print "Which arm: "
        arm = STDIN.gets
        arm.chomp!
        print "Which object: "
        object = STDIN.gets
        object.chomp!
        print avg_for_arm_object( data, arm, object )
        print "\n\n" 
      else
        print "Error, I did not understand your command!\n"
      end
    end

end

main()
