// Copyright 2007-2010 Baptiste Lepilleur
// Distributed under MIT license, or public domain if desired and
// recognized in your jurisdiction.
// See file LICENSE for detail or copy at http://jsoncpp.sourceforge.net/LICENSE

/* This executable is used for testing parser/writer using real JSON files.
 */

#include <json/read_file.h>


#if defined(_MSC_VER) && _MSC_VER >= 1310
#pragma warning(disable : 4996) // disable fopen deprecation warning
#endif

bool is_order = 0;
bool is_bin_name = 0;
int order_number = 0;

static std::string normalizeFloatingPointStr(double value) {
  char buffer[32];
#if defined(_MSC_VER) && defined(__STDC_SECURE_LIB__)
  sprintf_s(buffer, sizeof(buffer), "%.16g", value);
#else
  snprintf(buffer, sizeof(buffer), "%.16g", value);
#endif
  buffer[sizeof(buffer) - 1] = 0;
  std::string s(buffer);
  std::string::size_type index = s.find_last_of("eE");
  if (index != std::string::npos) {
    std::string::size_type hasSign =
        (s[index + 1] == '+' || s[index + 1] == '-') ? 1 : 0;
    std::string::size_type exponentStartIndex = index + 1 + hasSign;
    std::string normalized = s.substr(0, exponentStartIndex);
    std::string::size_type indexDigit =
        s.find_first_not_of('0', exponentStartIndex);
    std::string exponent = "0";
    if (indexDigit !=
        std::string::npos) // There is an exponent different from 0
    {
      exponent = s.substr(indexDigit);
    }
    return normalized + exponent;
  }
  return s;
}

static std::string readInputTestFile(const char* path) {
  FILE* file = fopen(path, "rb");
  if (!file){
  	std::cout << "path: " << path << std::endl;
    return std::string("");
}
  fseek(file, 0, SEEK_END);
  long size = ftell(file);
  fseek(file, 0, SEEK_SET);
  std::string text;
  char* buffer = new char[size + 1];
  buffer[size] = 0;
  if (fread(buffer, 1, size, file) == (unsigned long)size)
    text = buffer;
  fclose(file);
  delete[] buffer;
  return text;
}

static int
printValueTree(FILE* fout, Json::Value& value, WorkOrder& work_order, BinContents& bin_contents, const std::string& path = ".") {

  if (value.hasComment(Json::commentBefore)) {
    fprintf(fout, "%s\n", value.getComment(Json::commentBefore).c_str());
  }
  switch (value.type()) {
  case Json::nullValue:
    fprintf(fout, "%s=null\n", path.c_str());
    break;
  case Json::intValue:
    fprintf(fout,
            "%s=%s\n",
            path.c_str(),
            Json::valueToString(value.asLargestInt()).c_str());
    break;
  case Json::uintValue:
    fprintf(fout,
            "%s=%s\n",
            path.c_str(),
            Json::valueToString(value.asLargestUInt()).c_str());
    break;
  case Json::realValue:
    fprintf(fout,
            "%s=%s\n",
            path.c_str(),
            normalizeFloatingPointStr(value.asDouble()).c_str());
    break;
  case Json::stringValue:
    fprintf(fout, "%s=\"%s\"\n", path.c_str(), value.asString().c_str());
    break;
  case Json::booleanValue:
    fprintf(fout, "%s=%s\n", path.c_str(), value.asBool() ? "true" : "false");
    break;
  case Json::arrayValue: {
    fprintf(fout, "%s=[]\n", path.c_str());
    int size = value.size();
    for (int index = 0; index < size; ++index) {
     static char buffer[16];
#if defined(_MSC_VER) && defined(__STDC_SECURE_LIB__)
      sprintf_s(buffer, sizeof(buffer), "[%d]", index);
#else
      snprintf(buffer, sizeof(buffer), "[%d]", index);
#endif

    std::string bin_name = path.substr(path.length()-5, 5);;
    if (bin_name.compare("bin_A") == 0){
      bin_contents.bin_A[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_B") == 0){
      bin_contents.bin_B[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_C") == 0){
      bin_contents.bin_C[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_D") == 0){
      bin_contents.bin_D[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_E") == 0){
      bin_contents.bin_E[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_F") == 0){
      bin_contents.bin_F[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_G") == 0){
      bin_contents.bin_G[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_H") == 0){
      bin_contents.bin_H[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_I") == 0){
      bin_contents.bin_I[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_J") == 0){
      bin_contents.bin_J[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_K") == 0){
      bin_contents.bin_K[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_L") == 0){
      bin_contents.bin_L[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_M") == 0){
      bin_contents.bin_M[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_N") == 0){
      bin_contents.bin_N[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_O") == 0){
      bin_contents.bin_O[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_P") == 0){
      bin_contents.bin_P[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_Q") == 0){
      bin_contents.bin_Q[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_R") == 0){
      bin_contents.bin_R[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_S") == 0){
      bin_contents.bin_S[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_T") == 0){
      bin_contents.bin_T[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_U") == 0){
      bin_contents.bin_U[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_V") == 0){
      bin_contents.bin_V[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_W") == 0){
      bin_contents.bin_W[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_X") == 0){
      bin_contents.bin_X[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_Y") == 0){
      bin_contents.bin_Y[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("bin_Z") == 0){
      bin_contents.bin_Z[index] = value[index].asString().c_str();
    }
    else if (bin_name.compare("order") == 0){

      is_order = 1;
      is_bin_name = 1;
      order_number = index;
    }

      printValueTree(fout, value[index], work_order, bin_contents, path + buffer);
    }
  } break;
  case Json::objectValue: {
    fprintf(fout, "%s={}\n", path.c_str());
    Json::Value::Members members(value.getMemberNames());
    std::sort(members.begin(), members.end());
    std::string suffix = *(path.end() - 1) == '.' ? "" : ".";
    
    for (Json::Value::Members::iterator it = members.begin();
         it != members.end();
         ++it) {
      const std::string& name = *it;


    if (is_order ==1 && is_bin_name == 1){
        work_order.bin[order_number] = value[name].asString().c_str();
        is_bin_name = 0;
        // std::cout << "Work Order: "<< std::endl;
        // std::cout << work_order.bin[order_number] << std::endl;
    }
    else if (is_order ==1 && is_bin_name == 0){
        work_order.item[order_number] = value[name].asString().c_str();
        is_order = 0;
        // std::cout << work_order.item[order_number] << std::endl;
        // std::cout << "done: " << work_order.done[order_number] << std::endl;
    }

      printValueTree(fout, value[name], work_order, bin_contents, path + suffix + name);
    }
  } break;
  default:
    break;
  }

  if (value.hasComment(Json::commentAfter)) {
    fprintf(fout, "%s\n", value.getComment(Json::commentAfter).c_str());
  }

  return(order_number);
}

static int parseAndSaveValueTree(const std::string& input,
                                 const std::string& actual,
                                 const std::string& kind,
                                 Json::Value& root,
                                 const Json::Features& features,
                                 bool parseOnly,
                                 WorkOrder& work_order,
                                BinContents& bin_contents) {
  Json::Reader reader(features);
  bool parsingSuccessful = reader.parse(input, root);
  if (!parsingSuccessful) {
    printf("Failed to parse %s file: \n%s\n",
           kind.c_str(),
           reader.getFormattedErrorMessages().c_str());
  }

  if (!parseOnly) {
    FILE* factual = fopen(actual.c_str(), "wt");
    if (!factual) {
      printf("Failed to create %s actual file.\n", kind.c_str());
    }
    const std::string& path = "."; 

    order_number = printValueTree(factual, root, work_order, bin_contents, path);
    fclose(factual);
  }
  return(order_number);
}
void count_items_in_bins(BinContents& bin_contents, WorkOrder& work_order, int num_items)
{
    int bin_index = 0;
    bool found = false;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_A[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_B[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_C[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_D[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_E[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_F[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_G[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_H[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_I[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_J[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_K[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }
    found = false;
    bin_index++;
    for( int i = 0 ; i < 5 && !found ; i++ )
    {
        if(bin_contents.bin_L[i] == "")
        {
            bin_contents.item_counts[bin_index] = i;
            found = true;
        }
    }

    for(int i=0;i<num_items;i++)
    {
      work_order.item_counts[i] = bin_contents.item_counts[(work_order.bin[i][4]-'A')];
    }



}
static int rewriteValueTree(const std::string& rewritePath,
                            const Json::Value& root,
                            std::string& rewrite) {
  Json::StyledWriter writer;
  rewrite = writer.write(root);
  FILE* fout = fopen(rewritePath.c_str(), "wt");
  if (!fout) {
    printf("Failed to create rewrite file: %s\n", rewritePath.c_str());
    return 2;
  }
  fprintf(fout, "%s\n", rewrite.c_str());
  fclose(fout);
  return 0;
}

static std::string removeSuffix(const std::string& path,
                                const std::string& extension) {
  if (extension.length() >= path.length())
    return std::string("");
  std::string suffix = path.substr(path.length() - extension.length());
  if (suffix != extension)
    return std::string("");
  return path.substr(0, path.length() - extension.length());
}

static void printConfig() {
// Print the configuration used to compile JsonCpp
#if defined(JSON_NO_INT64)
  printf("JSON_NO_INT64=1\n");
#else
  printf("JSON_NO_INT64=0\n");
#endif
}

static int printUsage(const char* argv[]) {
  printf("Usage: %s [--strict] input-json-file", argv[0]);
  return 3;
}

int parseCommandLine(int argc,
                     const char* argv[],
                     Json::Features& features,
                     std::string& path,
                     bool& parseOnly) {
  parseOnly = false;
  if (argc < 2) {
    return printUsage(argv);
  }

  int index = 1;
  if (std::string(argv[1]) == "--json-checker") {
    features = Json::Features::strictMode();
    parseOnly = true;
    ++index;
  }

  if (std::string(argv[1]) == "--json-config") {
    printConfig();
    return 3;
  }

  if (index == argc || index + 1 < argc) {
    return printUsage(argv);
  }

  path = argv[index];
  return 0;
}

int read_file(std::string& file_name, WorkOrder& work_order, BinContents& bin_contents) {

  Json::Features features;
  bool parseOnly = false;

  try {
    std::string input = readInputTestFile(file_name.c_str());
    if (input.empty()) {
      printf("Failed to read input or empty input.");
    }

    std::string basePath = removeSuffix(file_name, ".json");
    if (basePath.empty()) {
      printf("Bad input path. Path does not end with '.expected");
    }

    std::string actualPath = basePath + ".actual";

    Json::Value root;
    order_number = parseAndSaveValueTree(
        input, actualPath, "input", root, features, parseOnly, work_order, bin_contents);
  }
  catch (const std::exception& e) {
    printf("Unhandled exception:\n%s\n", e.what());
  }
  return(order_number);
}


int read_preferences(std::string& file_name, ItemPreferences& i_pref)
{
  std::ifstream fin;
  fin.open(file_name.c_str());

  for(int i=0;i<NUM_ITEMS;i++)
  {
    std::string item_name;
    unsigned pref;
    int priority;
    std::getline( fin, item_name, ' ' );
    fin>>pref;
    fin>>priority;
    i_pref.items[i] = item_name;
    i_pref.gripper[i] = (PREFERENCE)pref;
    i_pref.priority[i] = priority;
    std::getline( fin, item_name );
    // std::cout<<i_pref.items[i]<<" "<<i_pref.gripper[i]<<std::endl;
  }

  fin.close();
}

