#include <netcdf>
#include "utils/data_loader.h"

namespace utils {

  DataLoader::DataLoader(const utils::Parameters::Ptr& pParams) {
    this->pParams = pParams;
    
    //load common params
    loadParams();
  }

  DataLoader::~DataLoader(void) {
    if (u_gps.size() > 0) {
      deleteGPs(u_gps);
    }
    if (v_gps.size() > 0) {
      deleteGPs(v_gps);
    }
  }


  void DataLoader::loadParams(void){
    // NC file
    std::string data_location = pParams->getParamNode()["imported_vf"]["file_location"].as<string>();
    file_name = data_location + pParams->getParamNode()["imported_vf"]["file_name"].as<std::string>();
    //cout<<"file name: "<<file_name<<endl;
    num_of_files = pParams->getParamNode()["imported_vf"]["num_of_files"].as<unsigned int>();
    scaling_x = pParams->getParamNode()["imported_vf"]["rescale_nc_x"].as<double>();
    scaling_y = pParams->getParamNode()["imported_vf"]["rescale_nc_y"].as<double>();

    // GP parameters
    input_dim = pParams->getParamNode()["imported_vf"]["gp_params"]["input_dim"].as<unsigned int>();
    cov_func = pParams->getParamNode()["imported_vf"]["gp_params"]["cov_func"].as<std::string>();
    cov_params.push_back(pParams->getParamNode()["imported_vf"]["gp_params"]["cov_params1"].as<double>());
    cov_params.push_back(pParams->getParamNode()["imported_vf"]["gp_params"]["cov_params2"].as<double>());
  }


  void DataLoader::importVecFieldData() {
    // Reading nc file
    size_t found = file_name.find_last_of("/\\"); // the last slash '/'
    unsigned int time_stamp = -1;
    size_t offset_len = 5; //the pos of first number
    size_t substr_len = 10; //the number string len
    if (found != std::string::npos) {
      //cout<<"str: "<<file_name.substr(found + offset_len, substr_len)<<endl; 
      time_stamp = std::stoi(file_name.substr(found + offset_len, substr_len));
    }

    unsigned int lat_size = 0;
    unsigned int lon_size = 0;
    float *lat_data = nullptr;
    float *lon_data = nullptr;

    for (int i = 0; i < num_of_files; i++) {
      try {
        std::string new_file_name = file_name;
        if (found != std::string::npos) {
          new_file_name.replace(found + offset_len, substr_len, std::to_string(time_stamp));
        }
        std::cout << "new_file_name " << new_file_name << std::endl;
        const netCDF::NcFile nc_file(new_file_name, netCDF::NcFile::read);
        time_stamp += 21600 * (119 / (num_of_files-1)); // intervals
        
        if (i == 0) {
          // latitude: float
          const netCDF::NcVar lat_var = nc_file.getVar("lat");
          lat_size = lat_var.getDim(0).getSize();
          lat_data = new float[lat_size];
          lat_var.getVar(lat_data);
          latitudes.clear();
          latitudes.insert(latitudes.end(), &lat_data[0], &lat_data[lat_size]);
          if (scaling_y != 1.0) {
            latitudes = resampling1D(latitudes, scaling_y);
          }
          // longitude: float
          const netCDF::NcVar lon_var = nc_file.getVar("lon");
          lon_size = lon_var.getDim(0).getSize();
          lon_data = new float[lon_size];
          lon_var.getVar(lon_data);
          longitudes.clear();
          longitudes.insert(longitudes.end(), &lon_data[0], &lon_data[lon_size]);
          if (scaling_x != 1.0) {
            longitudes = resampling1D(longitudes, scaling_x);
          }
        }

        raw_nc_data.n_rows = lat_size;
        raw_nc_data.n_cols = lon_size;

        // time: float
        const netCDF::NcVar time_var = nc_file.getVar("time");
        const unsigned int time_size = time_var.getDim(0).getSize();
        float *time_data = new float[time_size];
        time_var.getVar(time_data);

        // u: time, (depth), lat, lon
        const netCDF::NcVar u_var = nc_file.getVar("u");
        // v: time, (depth), lat, lon
        const netCDF::NcVar v_var = nc_file.getVar("v");
        // (salt): time, (depth), lat, lon
        const netCDF::NcVar salt_var = nc_file.getVar("salt");

        unsigned int data_size = 1;
        for (int i = 0; i < u_var.getDimCount(); i++) {
          data_size *= u_var.getDim(i).getSize();
        }

        float *u_data = new float[data_size];
        std::fill_n(u_data, data_size, -9999);
        u_var.getVar(u_data);

        float *v_data = new float[data_size];
        std::fill_n(v_data, data_size, -9999);
        v_var.getVar(v_data);

        float *salt_data = new float[data_size];
        std::fill_n(salt_data, data_size, -9999);
        if (!salt_var.isNull()) {
          salt_var.getVar(salt_data);
        }

        frame_t frame(raw_nc_data.n_rows, raw_nc_data.n_cols);
        frame.time = time_data[0];
        for (unsigned int j = 0; j < lat_size; j++) {
          for (unsigned int k = 0; k < lon_size; k++) {
            float u = u_data[0 * lat_size * lon_size + j * lon_size + k];
            float v = v_data[0 * lat_size * lon_size + j * lon_size + k];
            float salt = salt_data[0 * lat_size * lon_size + j * lon_size + k];
            if (u == -9999) {
              u = NAN;
            }
            if (v == -9999) {
              v = NAN;
            }
            if (salt == -9999) {
              salt = NAN;
            }

            cell_t cell;
            cell.coord = gu::Vec2(lat_data[j], lon_data[k]);
            cell.ocean_vec = gu::Vec2(u, v);
            cell.salinity = salt;
            cell.temperature = NAN;

            frame.cells[j][k] = cell;
            ///////

            if (i == 0) {
              vec_field.push_back(cell.ocean_vec);
            }
          }
        }
        raw_nc_data.frames.push_back(frame);
        if (i == 0) {
          vec_field = resampling2D(vec_field, lon_size, lat_size, scaling_x, scaling_y);
        }

        delete [] u_data;
        delete [] v_data;
        delete [] salt_data;
        delete [] time_data;

      } catch (netCDF::exceptions::NcException e) {
        cerr << "read error, file: " << file_name << endl;
      }
    }

    delete [] lat_data;
    delete [] lon_data;
    if (scaling_x != 1.0 || scaling_y != 1.0) {
      nc_data = resamplingNC(raw_nc_data, scaling_x, scaling_y);
    }
    
    u_gps = generateGPs(nc_data, input_dim, cov_func, cov_params, true); // delete
    v_gps = generateGPs(nc_data, input_dim, cov_func, cov_params, false); // delete
    
  }

  void
  DataLoader::importMatrix(ifstream& input_file, vector<vector<double> >& matrix) {

    string line;
    vector<double> numstream;

    uint num_rows = 0;
    uint num_cols = 0;

    if (input_file.is_open()) {
      while (!input_file.eof()) {
        getline(input_file, line);

        uint local_num_cols = 0;
        vector<double> local_numstream;
        string word;

        stringstream parse(line);
        while (parse >> word) {
          //if comment line, ignore it
          if (word[0] == '#' || word[0] == '%')
            break;
          numstream.push_back(atof(word.c_str())); //if not a number, get zero 
          local_numstream.push_back(atof(word.c_str()));
          local_num_cols++;
        } //end inner while

        //double check if the dense matrix format is correct or not
        if (num_cols && local_num_cols && num_cols != local_num_cols) {
          cerr << endl << "Please input a correct matrix format!" << endl << endl;
          exit(0);
        }
        //update column number if everything looks normal
        if (local_num_cols)
          num_cols = local_num_cols;
        //update row number if everything looks normal
        if (line.length() && local_numstream.size())
          num_rows++;

      } //end outer while

      input_file.close();

      matrix.clear();
      //put elements into matrix
      //matrix.resize(num_rows, num_cols);
      matrix.resize(num_rows);
      for (uint i = 0; i < num_rows; i++)
        matrix[i].resize(num_cols);

      vector<double>::iterator itr = numstream.begin();
      for (uint i = 0; i < num_rows; i++)
        for (uint j = 0; j < num_cols; j++)
          matrix[i][j] = *itr++;
    }//end outmost if
    else {
      cerr << endl << "Error: Unable to open file! Stopped." << endl << endl;
      exit(0);
    }

  }

  void
  DataLoader::importVec(ifstream& input_file, vector<double>& vec) {

    string line;

    if (input_file.is_open()) {
      while (!input_file.eof()) {
        uint num_cols = 0;
        vec.clear();
        getline(input_file, line);
        string word;

        stringstream parse(line);
        while (parse >> word) {
          //if comment line, ignore it
          if (word[0] == '#' || word[0] == '%')
            break;
          vec.push_back((double) atof(word.c_str())); //if not number, convert to zero
          num_cols++;
        } //end inner while

        if (num_cols > 0)
          break;

      } //end out layer while

      input_file.close();

    }//end outmost if
    else {
      cerr << endl << "Error: Unable to open file! Stopped." << endl << endl;
      exit(0);
    }

  }

  void
  DataLoader::displayMatrix(const vector<vector<double> >& m) {

    int DISPLAY_WIDTH = 10;
    if (m[0].size() > DISPLAY_WIDTH * 3) {
      _cout("Matrix is big, not displaying." << endl);
      return;
    }

    _cout(endl << "The matrix you queried is:" << endl << endl);
    for (uint i = 0; i < m.size(); i++) {
      for (uint j = 0; j < m[0].size(); j++)
        _cout("  " << m[i][j] << "\t");
      _cout(endl);
    }
    _cout(endl);

  }


}//namespace

