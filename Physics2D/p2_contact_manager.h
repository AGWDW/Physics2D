#pragma once
#include <array>
#include <vector>
#include "p2_circle_shape.h"
#include "p2_settings.h"
#include "p2_contact.h"

class p2ContactManager
{
public:
	/// <summary>
	/// Calls the broadphase and narrow phase
	/// </summary>
	/// <param name="shapes"></param>
	/// <returns>A list of all contacts that have occoured between the given shapes</returns>
	std::vector<p2Contact> GetContacts(std::array<p2CircleShape*, p2_Max_Objects> shapes);
private:
	/// <summary>
	/// copares the aabb's of each shape agais every other n^2 without repertions
	/// </summary>
	/// <param name="shapes"></param>
	/// <returns>all contacts that passed the broad phase</returns>
	std::vector<p2Contact> Broadphase(std::array<p2CircleShape*, p2_Max_Objects> shapes);
	/// <summary>
	/// edits the given contacts with more detailed infomation
	/// </summary>
	/// <param name="contacts"></param>
	void Narrowphase(std::vector<p2Contact>& contacts);
};

