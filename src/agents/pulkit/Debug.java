package agents.pulkit;

public class Debug{
	public static Boolean MODE = false;
	public static String allowOnly = "";

	public static void log(String category1, String message){
		if(MODE && (allowOnly.equals("") || allowOnly.contains(category1))) System.out.println(category1 + ": " + message);
	}

	public static void log(String category1, String category2, String message){
		if(MODE && (allowOnly.equals("") || allowOnly.contains(category1) || allowOnly.contains(category2))) System.out.println(category1
				+ ": " + category2 + ": " + message);
	}
}
