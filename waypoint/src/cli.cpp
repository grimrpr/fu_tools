#include "ros/ros.h"
#include "std_msgs/String.h"

#include "waypoint/StartRoute.h"
#include "waypoint/SaveRoute.h"
#include "waypoint/GetRoutes.h"

#include "waypoint/StartTrack.h"
#include "waypoint/MarkWaypoint.h"
#include "waypoint/SaveTrack.h"
#include "waypoint/StartRouteOfTrack.h"

#include "waypoint/GetTracks.h"
#include "waypoint/DeleteTrack.h"
#include "waypoint/DeleteRoute.h"

#include <sstream>
#include <iostream>

using namespace std;

ros::ServiceClient clientStartRoute;
ros::ServiceClient clientSaveRoute;
ros::ServiceClient clientGetRoutes;

ros::ServiceClient clientStartTrack;
ros::ServiceClient clientMarkWaypoint;
ros::ServiceClient clientSaveTrack;
ros::ServiceClient clientStartRouteOfTrack;

ros::ServiceClient clientGetTracks;
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

  return -1;

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
      waypoint::SaveTrack srv;
      srv.request.track = track;
      if(clientSaveTrack.call(srv))
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

  waypoint::StartTrack srv;
  srv.request.track.name = name;
  if(clientStartTrack.call(srv))
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

  waypoint::StartRouteOfTrack srv;
  srv.request.track = track;
  if(clientStartRouteOfTrack.call(srv))
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
    cout << "> Options for route " << route.time << endl;
    cout << "(0) Return to previous menu" << endl;
    cout << "(1) Delete route " << route.time << endl;
    cout << "> Enter number: ";
    unsigned int x = enter_number();
    if(x == 0)
    {
      return;
    }
    if(x == 1)
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

void show_menu_list_routes(waypoint::Track* track)
{

  while(ros::ok())
  {
    cout << endl;
    waypoint::GetRoutes srv;
    if(track != 0) {srv.request.track = (*track);}
    if(clientGetRoutes.call(srv))
    {
      std::vector<waypoint::Route> routes = srv.response.routes;
      if(routes.size() == 0)
      {
        cout << "> No routes found" << endl;
        return;
      }
      else
      {
        cout << "> Found " << routes.size() << " route(s)" << endl;
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
      show_menu_list_routes(&track);
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

void show_menu_new_route_without_track()
{
  while(ros::ok())
  {
    cout << endl;
    cout << "> Create new route without track" << endl;
    cout << "(0) Return to previous menu" << endl;
    cout << "(1) Start new route" << endl;
    unsigned int x = enter_number();
    if(x == 0) {return;}
    if(x == 1)
    {
      waypoint::StartRoute srv;
      if(clientStartRoute.call(srv))
      {
        if(srv.response.successful)
        {
          cout << endl;
          cout << "> Route was successfully started, drive around!" << endl;
          cout << "(0) Save current route" << endl;
          unsigned int x = enter_number();
          waypoint ::SaveRoute srv2;
          if(clientSaveRoute.call(srv2))
          {
            cout << "> Route " << srv2.response.route.time << " was successfully saved" << endl;
          }
          else
          {
            cerr << "> Service is not callable right now" << endl;
          }
        }
        else
        {
          cerr << "> Route could not be started" << endl;
        }
      }
      else
      {
        cerr << "> Service is not callable right now" << endl;
      }
    }
  }
}

void show_main_menu()
{

  while(ros::ok())
  {
    cout << "> Main menu" << endl;
    cout << "(0) Exit Program" << endl;
    cout << "(1) New route without track" << endl;
    cout << "(2) List routes without track" << endl;
    cout << "(3) New track" << endl;
    cout << "(4) List tracks" << endl;
    cout << "> Enter number: ";
    unsigned int x = enter_number();
    if(x == 0) {return;}
    if(x == 1) {show_menu_new_route_without_track();}
    if(x == 2) {show_menu_list_routes((waypoint::Track*) 0);}
    if(x == 3) {show_menu_new_track();}
    if(x == 4) {show_menu_list_tracks();}
    cout << endl;
  }

  return;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_cli");
  ros::NodeHandle n;

  clientStartRoute   = n.serviceClient<waypoint::StartRoute>("StartRoute");
  clientSaveRoute    = n.serviceClient<waypoint::SaveRoute>("SaveRoute");
  clientGetRoutes    = n.serviceClient<waypoint::GetRoutes>("GetRoutes");

  clientStartTrack   = n.serviceClient<waypoint::StartTrack>("StartTrack");
  clientMarkWaypoint = n.serviceClient<waypoint::MarkWaypoint>("MarkWaypoint");
  clientSaveTrack    = n.serviceClient<waypoint::SaveTrack>("SaveTrack");
  clientStartRouteOfTrack        = n.serviceClient<waypoint::StartRouteOfTrack>("StartRouteOfTrack");
 
  clientGetTracks       = n.serviceClient<waypoint::GetTracks>("GetTracks");
  clientDeleteTrack     = n.serviceClient<waypoint::DeleteTrack>("DeleteTrack");
  clientDeleteRoute     = n.serviceClient<waypoint::DeleteRoute>("DeleteRoute");

  show_main_menu();

}
