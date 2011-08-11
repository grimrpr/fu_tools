#include "ros/ros.h"
#include "std_msgs/String.h"
#include "waypoint/GetTracks.h"
#include "waypoint/GetRoutes.h"
#include "waypoint/StartNewTrack.h"
#include "waypoint/SaveNewTrack.h"
#include "waypoint/MarkWaypoint.h"
#include "waypoint/RunTrack.h"
#include "waypoint/PlayBack.h"
#include "waypoint/DeleteTrack.h"
#include "waypoint/DeleteRoute.h"

#include <sstream>
#include <iostream>

using namespace std;

ros::ServiceClient clientGetTracks;
ros::ServiceClient clientGetRoutes;
ros::ServiceClient clientStartNewTrack;
ros::ServiceClient clientSaveNewTrack;
ros::ServiceClient clientMarkWaypoint;
ros::ServiceClient clientRunTrack;
ros::ServiceClient clientPlayBack;
ros::ServiceClient clientDeleteTrack;
ros::ServiceClient clientDeleteRoute; 

std::string enter_name()
{

  std::string name;

  while(true)
  {
    if(cin >> name)
    {
      cin.clear();
      cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
      return name;
    }
  }

  return name;

}

unsigned int enter_number()
{

  unsigned int num;

  while(ros::ok())
  {
    if(cin >> num)
    {
      return num;
    }
    else
    {
      cerr << "> Not a number - try again: ";
      cin.clear();
      cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
  }

}

void show_menu_recording_track(waypoint::Track &track)
{

  while(ros::ok())
  {
    cout << endl;
    cout << "> Recording track " << track.name << endl;
    cout << "(0) Mark waypoint" << endl;
    cout << "(1) Save track" << endl;
    cout << "> Enter number: ";
    unsigned int x = enter_number();
    if(x == 0)
    {
      waypoint::MarkWaypoint srv;
      if(clientMarkWaypoint.call(srv))
      {
        if(srv.response.successful)
        {
          cout << "> Waypoint marked successfully" << endl;
        }
        else
        {
          cerr << "> Waypoint could not be marked" << endl;
        }
      }
      else
      {
        cerr << "> Service is not callable right now" << endl;
      }
    }
    if(x == 1)
    {
      waypoint::SaveNewTrack srv;
      srv.request.track = track;
      if(clientSaveNewTrack.call(srv))
      {
        if(srv.response.successful)
        {
          cout << "> Track saved successfully" << endl;
          return;
        }
        else
        {
          cerr << "> Track could not be saved" << endl;
        }
      }
      else
      {
        cerr << "> Service is not callable right now" << endl;
      }
    }
  }

}

void show_menu_new_track()
{

  cout << endl;
  cout << "> Enter name without whitespaces to start a new track: "; 
  std::string name = enter_name();

  waypoint::StartNewTrack srv;
  srv.request.track.name = name;
  if(clientStartNewTrack.call(srv))
  {
    if(srv.response.successful)
    {
      cout << "> Track started successfully" << endl;
      show_menu_recording_track(srv.response.track);
    }
    else
    {
      cerr << "> Track could not be started" << endl;
      return;
    }
  }
  else
  {
    cerr << "> Service is not callable right now" << endl;
    return;
  }

}

void show_menu_run_track(waypoint::Track &track)
{

  waypoint::RunTrack srv;
  srv.request.track = track;
  if(clientRunTrack.call(srv))
  {
    if(srv.response.successful)
    {
      cout << "> Track is now running" << endl;
      return;
    }
    else
    {
      cerr << "> Track could not be run" << endl;
      return;
    }
  }
  else
  {
    cerr << "> Service is not callable right now" << endl;
    return;
  }

}

void show_menu_specific_route(waypoint::Route &route)
{

  while(ros::ok())
  {
    cout << endl;
    cout << "> Options for route " << route.time << " of track " << route.name << endl;
    cout << "(0) Return to previous menu" << endl;
    cout << "(1) Play route " << route.time << " of track " << route.name << endl;
    cout << "(2) Delete route " << route.time << " of track " << route.name << endl;
    cout << "> Enter number: ";
    unsigned int x = enter_number();
    if(x == 0)
    {
      return;
    }
    if(x == 1)
    {
      waypoint::PlayBack srv;
      srv.request.route = route;
      if(clientPlayBack.call(srv))
      {
        if(srv.response.successful)
        {
          cout << "> Route is now playing" << endl;
        }
        else
        {
          cerr << "> Route could not be played" << endl;
        }
      }
      else
      {
        cerr << "> Service is not callable right now" << endl;
      }
    }
    if(x == 2)
    {
      waypoint::DeleteRoute srv;
      srv.request.route = route;
      if(clientDeleteRoute.call(srv))
      {
        if(srv.response.successful)
        {
          cerr << "> Route deleted" << endl;
          return;
        }
        else
        {
          cerr << "> Route could not be deleted" << endl;
        }
      }
      else
      {
        cerr << "> Service is not callable right now" << endl;
      }
    }
  }

}

void show_menu_list_routes(waypoint::Track &track)
{

  while(ros::ok())
  {
    cout << endl;
    waypoint::GetRoutes srv;
    srv.request.track = track;
    if(clientGetRoutes.call(srv))
    {
      std::vector<waypoint::Route> routes = srv.response.routes;
      if(routes.size() == 0)
      {
        cout << "> No routes found in track " << track.name << endl;
        return;
      }
      else
      {
        cout << "> Found " << routes.size() << " route(s) in track " << track.name << endl;
        cout << "(0) Return to previous menu" << endl;
        for(unsigned int i = 0; i < routes.size(); ++i)
        {
          cout << "(" << (i+1) << ") " << routes[i].time << endl; 
        }
        cout << "> Enter number: ";
        unsigned int x = enter_number();
        if(x == 0)
        {
          return;
        }
        if(x <= routes.size() && x > 0)
        {
          show_menu_specific_route(routes[x-1]);
        }
      } 
    }
    else
    {
      cerr << "> Service is not callable right now" << endl;
      return;  
    }
  }

}

void show_menu_specific_track(waypoint::Track &track)
{
  
  while(ros::ok())
  {
    cout << endl;
    cout << "> Options for track " << track.name << endl;
    cout << "(0) Return to previous menu" << endl;
    cout << "(1) Run track " << track.name << endl;
    cout << "(2) Show routes of track " << track.name << endl;
    cout << "(3) Delete track " << track.name << " and all of its routes" << endl;
    cout << "> Enter number: ";
    unsigned int x = enter_number();
    if(x == 0)
    {
      return;
    }
    if(x == 1)
    {
      show_menu_run_track(track);
    }
    if(x == 2)
    {
      show_menu_list_routes(track);
    }
    if(x == 3)
    {
      waypoint::DeleteTrack srv;
      srv.request.track = track;
      if(clientDeleteTrack.call(srv))
      {
        if(srv.response.successful)
        {
          cout << "> Track and its routes were deleted" << endl;
          return;
        }
        else
        {
          cerr << "> Track could not be deleted" << endl;
        }
      }
      else
      {
        cerr << "> Service is not callable right now" << endl;
      }
    }
  }

}

void show_menu_list_tracks(){
  
  while(ros::ok())
  {
    cout << endl;
    waypoint::GetTracks srv;
    if(clientGetTracks.call(srv))
    {
      std::vector<waypoint::Track> tracks = srv.response.tracks;
      if(tracks.size() == 0)
      {
        cerr << "> No tracks were found" << endl;
        return;
      }
      else
      {
        cout << "> Found " << tracks.size() << " Track(s)" << endl;
        cout << "(0) Return to previous menu" << endl;
        for(unsigned int i = 0; i < tracks.size(); ++i)
        {
          cout << "(" << (i+1) << ") " << tracks[i].name << endl;
        }
        cout << "> Enter number: ";
        unsigned int x = enter_number();
        if(x == 0)
        {
          return;
        }
        if(x <= tracks.size() && x > 0)
        {
          show_menu_specific_track(tracks[x-1]);
        }
      }
    }
    else
    {
      cerr << "> Service is not callable right now" << endl;
      return;
    }
  }

}

void show_main_menu()
{

  while(ros::ok())
  {
    cout << "> Main menu" << endl;
    cout << "(0) Exit Program" << endl;
    cout << "(1) New track" << endl;
    cout << "(2) List tracks" << endl;
    cout << "> Enter number: ";
    unsigned int x = enter_number();
    if(x == 0) {return;}
    if(x == 1) {show_menu_new_track();}
    if(x == 2) {show_menu_list_tracks();}
    cout << endl;
  }

  return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_cli");
	ros::NodeHandle n;
  
  clientGetTracks       = n.serviceClient<waypoint::GetTracks>("GetTracks");
  clientGetRoutes       = n.serviceClient<waypoint::GetRoutes>("GetRoutes");
  clientStartNewTrack   = n.serviceClient<waypoint::StartNewTrack>("StartNewTrack");
  clientSaveNewTrack    = n.serviceClient<waypoint::SaveNewTrack>("SaveNewTrack");
  clientMarkWaypoint    = n.serviceClient<waypoint::MarkWaypoint>("MarkWaypoint");
  clientRunTrack        = n.serviceClient<waypoint::RunTrack>("RunTrack");
  clientPlayBack        = n.serviceClient<waypoint::PlayBack>("PlayBack");
  clientDeleteTrack     = n.serviceClient<waypoint::DeleteTrack>("DeleteTrack");
  clientDeleteRoute     = n.serviceClient<waypoint::DeleteRoute>("DeleteRoute");

  show_main_menu();

}
